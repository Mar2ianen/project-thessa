// Volumetric cloud-deck march (`docs/14` section 10, `thessa-atmosphere::clouds`).
//
// A unit-sphere shell mesh is COVERAGE ONLY: every pixel marches a view ray
// through the planet-normalized slab ([bottom_frac, top_frac] planet radii)
// and Beer-Lambert-integrates a Schneider-style (Horizon Zero Dawn) density
// field built from the deck bake the shells show (EVE-style weather map) plus
// fully GPU-side weather: cloud family and deck-height levels from
// low-frequency procedural noise per pixel (stratus sheets, cumulus, towers),
// wind shear with altitude, coverage remap, type-dependent height gradient,
// subtractive billow-detail erosion. Lighting is Beer-Powder sun
// through-scatter (puff sides facing the sun glow, deep cores stay dark,
// low sun burns orange) plus a dual-lobe phase (forward silver lining) plus
// a height-graded ambient (dark blue-grey base, bright top, scaled by
// running transmittance so cores darken inside).
//
// Math mirrors `crates/atmosphere/src/clouds.rs` (`remap_coverage`,
// `type_gradient`, `erode_density`, `beer_powder`, `dual_lobe_phase`,
// `ambient_rgb`); the CPU functions are the oracle and this pass must agree
// with them within representation error on the same density field.
//
// Marching in planet radii (not metres) keeps one shader for the compressed
// map view and true-scale pilot views: extinction is per radii, appearance
// only, like `density_scale`.
//
// Cost (worst case, all steps resident): N view steps x (1 2D weather bake tap +
// 3 sun shadow taps when occupied). Dense cores early-out on transmittance (< 1e-4).
// Per-step weather bake test (< 0.02) skips expensive 3D procedural noise on clear steps.
// N <= CLOUD_MARCH_MAX (24), resolved from `clouds.ray_steps` (`crates/graphics`).

import bevy_pbr::render::mesh_view_bindings::view;
import bevy_pbr::render::forward_io::VertexOutput;

struct CloudUniforms {
    // planet center (world) + planet radius (world units).
    center_radius: vec4<f32>,
    // slab bottom/top (planet-radii fracs), extinction (1/radii), steps.
    slab: vec4<f32>,
    // sun direction (world, toward the sun) + eclipse visibility 0..1.
    sun: vec4<f32>,
    // deck-world to deck-local rotation (inverse of the deck spin the shell
    // follower applies), so bake UVs track the visible impostors exactly.
    deck_to_local: vec4<f32>,
    // opacity gain, pre-wrapped wind phase (UV units, f64-exact on CPU),
    // sun-tap span (slab fractions), detail perturbation amplitude.
    params: vec4<f32>,
    // deck type (0 = low cumulus, 1 = high cirrus), coverage floor,
    // impostor mesh scale (radii fracs, far-side cull), -.
    shape: vec4<f32>,
}

// Material bind group is 3 in Bevy 0.20 (MATERIAL_BIND_GROUP_INDEX), NOT 2.
@group(3) @binding(0)
var<uniform> cloud: CloudUniforms;
@group(3) @binding(1)
var coverage_tex: texture_2d<f32>;
@group(3) @binding(2)
var coverage_sampler: sampler;

// Keep in sync with `resolve.rs` CLOUD_MARCH_STEP_CAP.
const CLOUD_MARCH_MAX: i32 = 24;

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

// Inverse of the Bevy Sphere mesh mapping (poles on local +Z, u from +X
// toward +Y, v = 0 at the +Z pole). Mirrors the CPU `shell_mesh_uv` in the
// client and `shell_mesh_uv` in the terrain shader; keep the three in sync.
fn shell_mesh_uv(local_dir: vec3<f32>) -> vec2<f32> {
    let u = fract(atan2(local_dir.y, local_dir.x) / 6.2831853);
    let v = 0.5 - asin(clamp(local_dir.z, -1.0, 1.0)) / 3.14159265;
    return vec2(u, clamp(v, 0.0, 1.0));
}

// Forward-scattering powder component matching Rust thessa-atmosphere::clouds:
fn powder_scatter_term(ld: f32) -> f32 {
    let d = max(ld, 0.0);
    return exp(-d) * (1.0 - exp(-d * 2.0));
}

// Physical transmission with forward-scattering powder rim:
// Beer attenuates through the volume, while powder scattering boosts forward
// rims so sunlit cauliflower tops glow brilliantly and deep cores model dark.
fn beer_powder(ld: f32) -> f32 {
    let d = max(ld, 0.0);
    let beer = exp(-d);
    return max(beer, 2.0 * powder_scatter_term(d));
}

fn hg_phase(g: f32, cos_theta: f32) -> f32 {
    let g2 = g * g;
    let denom = pow(max(1.0 + g2 - 2.0 * g * cos_theta, 1e-4), 1.5);
    return (1.0 - g2) / (12.566371 * denom);
}

fn dual_lobe_phase(cos_theta: f32) -> f32 {
    return 0.70 * hg_phase(0.76, cos_theta) + 0.30 * hg_phase(-0.30, cos_theta);
}

// Convective cumulus height gradient (Schneider / LCL physics) matching Rust:
fn convective_cumulus_gradient(h01: f32) -> f32 {
    let base_lcl = smoothstep(0.065, 0.082, h01);
    let dome_top = 1.0 - smoothstep(0.72, 0.98, h01);
    let updraft = 1.0 + 0.40 * sin(clamp((h01 - 0.065) / 0.915, 0.0, 1.0) * 3.14159265);
    let hgrad = base_lcl * dome_top * updraft;
    if (hgrad <= 0.001) {
        return 0.0;
    }
    return hgrad;
}

fn hash12(p: vec2<f32>) -> f32 {
    var p3 = fract(vec3<f32>(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

fn hash13(p: vec3<f32>) -> f32 {
    let q = fract(p * 0.1031);
    let dot_q = dot(q, q.yzx + 33.33);
    return fract((q.x + q.y) * q.z + dot_q);
}

fn hash33(p: vec3<f32>) -> vec3<f32> {
    var q = fract(p * vec3<f32>(0.1031, 0.1030, 0.0973));
    q += dot(q, q.yzx + 33.33);
    return fract(vec3<f32>(
        (q.x + q.y) * q.z,
        (q.x + q.z) * q.y,
        (q.y + q.z) * q.x
    )) * 2.0 - 1.0;
}

fn hash_grad(p: vec3<f32>) -> vec3<f32> {
    let h = hash33(p);
    return normalize(h + vec3<f32>(1e-5, 1e-5, 1e-5));
}

// 3D Perlin gradient noise with quintic interpolation (C^2 smooth, zero lattice artifacts):
fn perlin3d(p: vec3<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);

    let g000 = hash_grad(i + vec3<f32>(0.0, 0.0, 0.0));
    let g100 = hash_grad(i + vec3<f32>(1.0, 0.0, 0.0));
    let g010 = hash_grad(i + vec3<f32>(0.0, 1.0, 0.0));
    let g110 = hash_grad(i + vec3<f32>(1.0, 1.0, 0.0));
    let g001 = hash_grad(i + vec3<f32>(0.0, 0.0, 1.0));
    let g101 = hash_grad(i + vec3<f32>(1.0, 0.0, 1.0));
    let g011 = hash_grad(i + vec3<f32>(0.0, 1.0, 1.0));
    let g111 = hash_grad(i + vec3<f32>(1.0, 1.0, 1.0));

    let v000 = dot(g000, f - vec3<f32>(0.0, 0.0, 0.0));
    let v100 = dot(g100, f - vec3<f32>(1.0, 0.0, 0.0));
    let v010 = dot(g010, f - vec3<f32>(0.0, 1.0, 0.0));
    let v110 = dot(g110, f - vec3<f32>(1.0, 1.0, 0.0));
    let v001 = dot(g001, f - vec3<f32>(0.0, 0.0, 1.0));
    let v101 = dot(g101, f - vec3<f32>(1.0, 0.0, 1.0));
    let v011 = dot(g011, f - vec3<f32>(0.0, 1.0, 1.0));
    let v111 = dot(g111, f - vec3<f32>(1.0, 1.0, 1.0));

    let vx00 = mix(v000, v100, u.x);
    let vx10 = mix(v010, v110, u.x);
    let vx01 = mix(v001, v101, u.x);
    let vx11 = mix(v011, v111, u.x);
    let vxy0 = mix(vx00, vx10, u.y);
    let vxy1 = mix(vx01, vx11, u.y);
    return mix(vxy0, vxy1, u.z);
}

// Irrational 3D rotation matrix 1 (rotation by ~37 degrees around [1, 2, 3]):
const ROT_OCTAVE = mat3x3<f32>(
    vec3<f32>( 0.00,  0.80,  0.60),
    vec3<f32>(-0.80,  0.36, -0.48),
    vec3<f32>(-0.60, -0.48,  0.64)
);

// Irrational 3D rotation matrix 2 (rotation by ~63 degrees around [2, -3, 1]):
// Destroys any constructive lattice interference or Cartesian quadrant alignment between octaves
const ROT_OCTAVE_2 = mat3x3<f32>(
    vec3<f32>(-0.573576,  0.707107, -0.414214),
    vec3<f32>(-0.573576, -0.707107, -0.414214),
    vec3<f32>( 0.585501,  0.000000, -0.810672)
);

// Procedural atmospheric domain warp (swirl turbulence):
// Empirical coordinate displacement (not mathematically divergence-free):
fn curl_warp3d(p: vec3<f32>) -> vec3<f32> {
    let w0 = perlin3d(p);
    let w1 = perlin3d(p + vec3<f32>(3.71, 1.29, 5.87));
    let w2 = perlin3d(p + vec3<f32>(8.13, 6.47, 2.31));
    return vec3<f32>(w1 - w2, w2 - w0, w0 - w1);
}

// C^1 Smooth cubic billow lobe:
// Flat derivative at the peak (db/dn = 0), zero creases, zero lattice edge visibility:
fn billow_cubic(p: vec3<f32>) -> f32 {
    let n = perlin3d(p);
    return clamp(1.0 - 1.85 * n * n, 0.0, 1.0);
}

const BILLOW_MEAN: f32 = 0.933;

// Hierarchical cauliflower billow field with rotational scrambling & fluid advection:
// Step-footprint filtered: higher octaves fade out toward verified empirical mean (~0.933) as raymarch
// step approaches each octave's Nyquist limit (lambda / 2), suppressing sub-step aliasing while
// maintaining expectation prior to downstream nonlinear density shaping.
fn cauliflower_fbm(p: vec3<f32>, step_km: f32) -> f32 {
    // Octave periods: ~14.0 km (b0), ~6.2 km (b1), ~2.8 km (b2)
    let w0 = smoothstep(7.0, 2.5, step_km);
    var b0 = BILLOW_MEAN;
    if (w0 > 0.01) {
        b0 = mix(BILLOW_MEAN, billow_cubic(p * 0.072), w0);
    }

    let w1 = smoothstep(3.1, 1.2, step_km);
    var b1 = BILLOW_MEAN;
    var b2 = BILLOW_MEAN;
    if (w1 > 0.01) {
        let p1 = ROT_OCTAVE * (p * 0.162) + vec3<f32>(1.73, 4.29, 8.11);
        b1 = mix(BILLOW_MEAN, billow_cubic(p1), w1);
        let w2 = smoothstep(1.4, 0.5, step_km);
        if (w2 > 0.01) {
            let p2 = ROT_OCTAVE_2 * (p1 * 2.21) + vec3<f32>(5.19, 1.87, 3.63);
            b2 = mix(BILLOW_MEAN, billow_cubic(p2), w2);
        }
    }
    // Weights sum to 1.0 (0.40 + 0.38 + 0.22 = 1.00); unresolvable octaves fade towards their statistical
    // expectation (0.933), maintaining the expectation of the linear field across LODs before nonlinear shaping:
    return b0 * 0.40 + b1 * 0.38 + b2 * 0.22;
}

// Sub-kilometer organic wispy turbulence bandlimited to ray step:
fn edge_wisps(p: vec3<f32>, step_km: f32) -> f32 {
    let w = smoothstep(1.5, 0.3, step_km);
    if (w <= 0.01) {
        return 0.50; // Neutral midpoint when unresolved
    }
    let q = p * 0.38 + vec3<f32>(2.31, 7.14, 4.43);
    let warp = curl_warp3d(q * 0.8) * 0.45;
    let n0 = perlin3d(q + warp) * 0.50 + 0.50;
    let q2 = ROT_OCTAVE_2 * (q + warp) * 1.85 + vec3<f32>(4.17, 9.31, 1.73);
    let n1 = perlin3d(q2) * 0.50 + 0.50;
    return mix(0.50, n0 * 0.70 + n1 * 0.30, w);
}

// 2D isotropic quasi-blue noise dither (plastic constant R2 dispersion):
// Eliminates directional diagonal striping and stippling on static single-frame raymarch edges:
fn dither2d(pos: vec2<f32>) -> f32 {
    let p = fract(pos * vec2<f32>(0.754877666, 0.56984029));
    let q = fract(vec2<f32>(p.y * 13.17, p.x * 19.31) + dot(p, p + 31.416));
    return fract(p.x + p.y + q.x * 0.5);
}

// Interleaved Gradient Noise kept for compatibility:
fn ign(pos: vec2<f32>) -> f32 {
    return dither2d(pos);
}

// Floored float modulus (WGSL has no `mod` builtin, and `%` truncates
// toward zero, which breaks lattice wraps for negative indices).
fn mod_pos(m: f32, p: f32) -> f32 {
    return m - p * floor(m / p);
}

fn vnoise(p: vec2<f32>, period: f32) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    var xi0 = i.x;
    var xi1 = i.x + 1.0;
    if (period > 0.5) {
        xi0 = mod_pos(xi0, period);
        xi1 = mod_pos(xi1, period);
    }
    let a = hash12(vec2<f32>(xi0, i.y));
    let b = hash12(vec2<f32>(xi1, i.y));
    let c = hash12(vec2<f32>(xi0, i.y + 1.0));
    let d = hash12(vec2<f32>(xi1, i.y + 1.0));
    return a + (b - a) * u.x + (c - a) * u.y + (a - b - c + d) * u.x * u.y;
}

// Deck-local bake UV for a planet-normalized point.
fn deck_uv(p_norm: vec3<f32>) -> vec2<f32> {
    let r = max(length(p_norm), 1e-6);
    let local = quat_rotate(cloud.deck_to_local, p_norm / r);
    return shell_mesh_uv(local);
}

// Base bake density at a deck-local UV: the EVE-style weather map (alpha).
fn base_at_uv(uv: vec2<f32>) -> f32 {
    return textureSampleLevel(coverage_tex, coverage_sampler, uv, 0.0).a;
}

fn base_at(p_norm: vec3<f32>) -> f32 {
    return base_at_uv(deck_uv(p_norm));
}

// GPU weather: cloud family + deck-height variation from low-frequency
// procedural noise (continent scale), evaluated per PIXEL at the interval
// ends and interpolated along the ray — never per step, never from CPU.
fn weather_at(p_norm: vec3<f32>, drift_uv: vec2<f32>) -> vec2<f32> {
    let uv = deck_uv(p_norm) + drift_uv;
    let type01 = vnoise(uv * vec2<f32>(5.0, 5.0) + vec2<f32>(7.3, 2.9), 5.0);
    let hvar = vnoise(uv * vec2<f32>(6.0, 6.0) + vec2<f32>(1.7, 9.1), 6.0);
    return vec2(type01, hvar);
}

// Wind shear with altitude: the whole column leans slightly with height.
const WIND_SHEAR = vec2<f32>(0.002, 0.001);

// 3D Volumetric density sample: metric kilometer coordinates, Schneider coverage remap,
// multi-octave 3D billow noise for spherical cauliflower cumulus lobes.
fn slab_density(
    p_norm: vec3<f32>,
    drift: vec2<f32>,
    weather: vec3<f32>,
    step_km: f32,
    full_detail: bool,
) -> f32 {
    let r = length(p_norm);
    let top_eff = cloud.slab.x + (cloud.slab.y - cloud.slab.x) * (0.65 + 0.70 * weather.z);
    let h01 = (r - cloud.slab.x) / max(top_eff - cloud.slab.x, 1e-6);
    if (h01 <= 0.0 || h01 >= 1.0) {
        return 0.0;
    }
    let uv = deck_uv(p_norm) + WIND_SHEAR * clamp(h01, 0.0, 1.0);
    let base = base_at_uv(uv);
    if (base < 0.02) {
        return 0.0;
    }

    let hgrad = convective_cumulus_gradient(h01);
    if (hgrad <= 0.0) {
        return 0.0;
    }

    // Physical position in deck-local metric kilometer coordinates (1.0 = 1 kilometer):
    let radius_km = max(cloud.center_radius.w, 1e-6) * 0.001;
    let p_km = quat_rotate(cloud.deck_to_local, p_norm) * radius_km;
    let p_drift = p_km + vec3<f32>(drift.x * 60.0, drift.y * 35.0, 0.0);

    // Cirrus deck (deck 1, shape.x > 0.5):
    if (cloud.shape.x > 0.5) {
        let p_cirrus = p_drift * vec3<f32>(0.25, 0.85, 0.25);
        let n1 = perlin3d(p_cirrus) * 0.50 + 0.50;
        let n2 = perlin3d(p_cirrus * 2.5 + vec3<f32>(5.1, 2.3, 7.7)) * 0.50 + 0.50;
        let wisp = n1 * 0.65 + n2 * 0.35;
        let d_cirrus = smoothstep(0.35, 0.75, base * wisp * hgrad) * 0.40 * cloud.params.x;
        return d_cirrus;
    }

    // Cumulus deck (deck 0):
    // Large atmospheric domain warp (swirl eddies, period ~45 km, Nyquist limit ~22 km):
    // Smoothly fades across 22.0 km to 8.0 km step sizes so coarse orbital steps (> 22 km)
    // do not alias high spatial frequencies, and saves 3x perlin3d calls on orbital chords.
    let w_warp_large = smoothstep(22.0, 8.0, step_km);
    var warp_large = vec3<f32>(0.0);
    if (w_warp_large > 0.0001) {
        warp_large = curl_warp3d(p_drift * 0.022) * (4.2 * w_warp_large);
    }

    if (!full_detail) {
        let p_w = p_drift + warp_large;
        let w_p0_s = smoothstep(36.0, 16.0, step_km);
        var p0_s = 0.50;
        if (w_p0_s > 0.001) {
            let p0_raw = perlin3d(p_w * 0.016) * 0.60 + 0.50;
            p0_s = mix(0.50, p0_raw, w_p0_s);
        }
        let w_macro_s = smoothstep(12.0, 4.0, step_km);
        var p1_s = 0.50;
        if (w_macro_s > 0.01) {
            let p1_raw = perlin3d(p_w * 0.038 + vec3<f32>(2.4, 5.1, 8.9)) * 0.50 + 0.50;
            p1_s = mix(0.50, p1_raw, w_macro_s);
        }
        let macro_s = p0_s * 0.70 + p1_s * 0.30;
        let w0_s = smoothstep(7.0, 2.5, step_km);
        var b0_s = BILLOW_MEAN;
        if (w0_s > 0.01) {
            b0_s = mix(BILLOW_MEAN, billow_cubic(p_w * 0.072), w0_s);
        }
        let w1_s = smoothstep(3.1, 1.2, step_km);
        var b1_s = BILLOW_MEAN;
        if (w1_s > 0.01) {
            let p1_s_rot = ROT_OCTAVE * (p_w * 0.162) + vec3<f32>(1.73, 4.29, 8.11);
            b1_s = mix(BILLOW_MEAN, billow_cubic(p1_s_rot), w1_s);
        }
        let cauli_s = b0_s * 0.51 + b1_s * 0.49;
        let billow_lift_s = mix(0.12, 0.60, smoothstep(0.08, 0.65, h01));
        let shape_s = macro_s * (1.0 - billow_lift_s) + cauli_s * billow_lift_s;
        let cov_s = clamp(base * mix(0.88, 1.12, weather.x), 0.0, 1.0);
        let cov_eff_s = clamp(cov_s * hgrad * 1.25, 0.0, 1.0);
        let thresh_s = clamp(1.0 - cov_eff_s, 0.05, 0.92);
        if (shape_s <= thresh_s) {
            return 0.0;
        }
        let rel_s = clamp((shape_s - thresh_s) / max(1.0 - thresh_s, 0.04), 0.0, 1.0);
        return smoothstep(0.0, 0.12, rel_s) * pow(rel_s, 1.1) * 6.2 * cloud.params.x;
    }

    // Small vortex turbulence (period ~12 km, Nyquist ~6 km): smoothly fades to 0 between 6 and 1.5 km steps
    let w_warp_turb = smoothstep(6.0, 1.5, step_km);
    var warp_turb = vec3<f32>(0.0);
    if (w_warp_turb > 0.0001) {
        warp_turb = curl_warp3d(p_drift * 0.085 + warp_large * 0.3) * (1.6 * w_warp_turb);
    }
    let p_w = p_drift + warp_large + warp_turb;

    // 1. Continuous continental cloud mass (Perlin macro connectivity, periods ~60 km and ~26 km):
    let w_p0 = smoothstep(36.0, 16.0, step_km);
    var p0 = 0.50;
    if (w_p0 > 0.001) {
        let p0_raw = perlin3d(p_w * 0.016) * 0.60 + 0.50;
        p0 = mix(0.50, p0_raw, w_p0);
    }
    // Bandlimit p1 (period ~26 km, Nyquist ~13 km) to its expectation (0.50) when step_km > 10 km:
    let w_macro = smoothstep(12.0, 4.0, step_km);
    var p1 = 0.50;
    if (w_macro > 0.01) {
        let p1_raw = perlin3d(p_w * 0.038 + vec3<f32>(2.4, 5.1, 8.9)) * 0.50 + 0.50;
        p1 = mix(0.50, p1_raw, w_macro);
    }
    let macro_perlin = p0 * 0.70 + p1 * 0.30;

    // 2. Hierarchical 3-octave cauliflower billow field with irrational rotations:
    let cauli = cauliflower_fbm(p_w, step_km);

    // 3. Convective cauliflower sculpting (modulated by cloud.params.w DETAIL_AMP):
    // Solid flat macro foundation at the base; buoyant cauliflower plumes bubbling upward.
    // Normalized by 1.6 baseline constant so default CLOUD_DETAIL_AMP yields 1.0 multiplier:
    let detail_amp = max(cloud.params.w / 1.6, 0.05);
    let billow_lift = mix(0.12, 0.60, smoothstep(0.08, 0.65, h01)) * detail_amp;
    let shape_body = macro_perlin * (1.0 - billow_lift) + cauli * billow_lift;

    // 4. Coverage remap (Schneider):
    let cov = clamp(base * mix(0.88, 1.12, weather.x), 0.0, 1.0);
    let cov_floor = clamp(cloud.shape.y, 0.0, 0.85);
    if (cov < cov_floor * 0.5) {
        return 0.0;
    }
    let cov_eff = clamp(cov * hgrad * 1.25, 0.0, 1.0);
    let threshold = clamp(1.0 - cov_eff, 0.05, 0.92);
    if (shape_body <= threshold) {
        return 0.0;
    }

    let base_cloud = (shape_body - threshold) / max(1.0 - threshold, 0.04);

    // 5. Edge-only Schneider erosion (bandlimited to step_km, modulated by detail_amp):
    // Crucial: erosion is suppressed at the condensation base (h01 < 0.12) to preserve a razor-flat horizontal shelf!
    let wisps = edge_wisps(p_w, step_km);
    let edge_factor = 1.0 - clamp(base_cloud, 0.0, 1.0);
    let base_erosion = smoothstep(0.09, 0.28, h01);
    let erosion = (1.0 - wisps) * (0.35 * base_erosion * edge_factor * detail_amp);
    let final_shape = max(base_cloud - erosion, 0.0);

    // 6. C^1 Continuous boundary power curve (crisp droplet boundary):
    let rel_d = clamp(final_shape, 0.0, 1.0);
    let d = smoothstep(0.0, 0.12, rel_d) * pow(rel_d, 1.1) * 6.2 * cloud.params.x;
    return d;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let center = cloud.center_radius.xyz;
    let radius = max(cloud.center_radius.w, 1e-6);
    let ro_world = view.world_position.xyz;

    // Single-sided coverage with a no-cull pipeline:
    // When camera is above the mesh bounding sphere, render front faces.
    // When camera is below/inside, render back faces (looking up at the sky).
    let cam_to_frag = in.world_position.xyz - ro_world;
    let dist_to_frag = length(cam_to_frag);
    if (dist_to_frag < 1e-4) {
        discard;
    }
    let rd_world = cam_to_frag / dist_to_frag;
    let to_cam = -rd_world;
    let surf_n = normalize(in.world_position.xyz - center);
    let r_cam = length(ro_world - center) / radius;
    let above = r_cam > cloud.shape.z;
    if ((dot(surf_n, to_cam) > 0.0) != above) {
        discard;
    }

    let ro = (ro_world - center) / radius;
    let rd = rd_world;

    let bot = cloud.slab.x;
    let top = cloud.slab.y;
    let march_top = cloud.shape.z;
    let ext = max(cloud.slab.z, 0.0);
    let steps = max(i32(cloud.slab.w + 0.5), 1);
    let drift = vec2<f32>(cloud.params.y, cloud.params.y * 0.32);

    let b = dot(ro, rd);

    // 1. Intersection with cloud ceiling sphere (radius = march_top)
    let disc_top = b * b - (r_cam * r_cam - march_top * march_top);
    if (disc_top < 0.0) {
        discard;
    }
    let sq_top = sqrt(disc_top);
    let t_top0 = -b - sq_top;
    let t_top1 = -b + sq_top;
    if (t_top1 <= 0.0) {
        discard;
    }

    var t0 = max(t_top0, 0.0);
    var t1 = t_top1;

    // 2. Intersection with inner sphere (radius = bot)
    let disc_bot = b * b - (r_cam * r_cam - bot * bot);
    if (disc_bot > 0.0) {
        let sq_bot = sqrt(disc_bot);
        let t_bot0 = -b - sq_bot;
        let t_bot1 = -b + sq_bot;

        if (r_cam < bot) {
            // Observer is BELOW the cloud deck (looking up from ground or lower atmosphere):
            // Ray enters slab at t_bot1, exits at t_top1.
            if (t_bot1 > 0.0) {
                t0 = max(t0, t_bot1);
            }
        } else {
            // Observer is ABOVE or INSIDE the cloud deck:
            // Ray enters hollow core at t_bot0, stopping the near slab march.
            if (t_bot0 > 0.0) {
                t1 = min(t1, t_bot0);
            }
        }
    }

    // 3. Occlusion by solid planet surface (radius = 1.0)
    let disc_planet = b * b - (r_cam * r_cam - 1.0);
    if (disc_planet > 0.0) {
        let t_planet0 = -b - sqrt(disc_planet);
        if (t_planet0 > 1e-5) {
            t1 = min(t1, t_planet0);
        } else if (b <= 0.0) {
            discard;
        }
    }

    // 4. Bound max march distance through the slab:
    // Continuous camera altitude budget policy:
    // Smoothly transitions between the low-altitude horizon budget cap (0.025 radii = 80 km on Thessa, R=3200 km)
    // and the full orbital geometric tangent chord (up to 868.8 km on Thessa) across the altitude range
    // from cloud ceiling (march_top ~ 54.7 km) to orbital insertion (~ 214.7 km, march_top + 0.05).
    // This maintains C^1 continuity, eliminating any step-boundary popping during ascent through 115-125 km.
    let geometric_tangent_chord = 2.0 * sqrt(max(march_top * march_top - bot * bot, 1e-4));
    let r_low = march_top;
    let r_high = march_top + 0.05;
    let altitude_weight = smoothstep(r_low, r_high, r_cam);
    let max_chord = mix(min(geometric_tangent_chord, 0.025), geometric_tangent_chord, altitude_weight);
    t1 = min(t1, t0 + max_chord);

    if (t1 <= t0 + 1e-5) {
        discard;
    }

    let sun_dir = normalize(cloud.sun.xyz + vec3<f32>(1e-9, 0.0, 0.0));
    let sun_vis = clamp(cloud.sun.w, 0.0, 1.0);
    let cos_theta = dot(rd, sun_dir);
    let thick = max(top - bot, 1e-6);
    let inv_radius = 1.0 / radius;
    // Sun shadow tap span: normalized by 0.8 baseline constant so default
    // CLOUD_SUN_SPAN_FRAC yields 1.0 multiplier (180 m / 650 m / 1800 m base taps):
    let shadow_span = max(cloud.params.z / 0.8, 0.05);

    let chord = max(t1 - t0, 1e-5);
    let w_entry = weather_at(ro + rd * t0, drift);
    let w_exit = weather_at(ro + rd * t1, drift);
    var transmittance = 1.0;
    var scat = vec3<f32>(0.0);
    var unsaturated_tau: f32 = 0.0;
    var steps_taken: f32 = 0.0;

    let radius_km = radius * 0.001;
    let p0_deck = quat_rotate(cloud.deck_to_local, ro + rd * t0) * radius_km;

    // Diagnostic Mode 2: Coverage Bake
    if (cloud.shape.w >= 1.5 && cloud.shape.w < 2.5) {
        let uv = deck_uv(ro + rd * t0) + WIND_SHEAR * 0.5;
        let base = base_at_uv(uv);
        return vec4<f32>(vec3<f32>(base), 1.0);
    }

    // Diagnostic Mode 7: Production-LOD Ray-Chord Midpoint Density Slice (evaluated at actual march step_km)
    if (cloud.shape.w >= 6.5 && cloud.shape.w < 7.5) {
        let t_mid = (t0 + t1) * 0.5;
        let p_mid = ro + rd * t_mid;
        let w_mid = mix(w_entry, w_exit, 0.5);
        let weather_mid = vec3<f32>(w_mid.x, 0.0, w_mid.y);
        let dt = chord / f32(steps);
        let step_km = dt * radius_km;
        let d_raw = slab_density(p_mid, drift, weather_mid, step_km, true);
        let d_norm = clamp(d_raw / max(6.2 * cloud.params.x, 1e-3), 0.0, 1.0);
        return vec4<f32>(vec3<f32>(d_norm), 1.0);
    }

    // Diagnostic Mode 8: High-Detail (1.0 km) Ray-Chord Midpoint Density Slice
    if (cloud.shape.w >= 7.5) {
        let t_mid = (t0 + t1) * 0.5;
        let p_mid = ro + rd * t_mid;
        let w_mid = mix(w_entry, w_exit, 0.5);
        let weather_mid = vec3<f32>(w_mid.x, 0.0, w_mid.y);
        let d_raw = slab_density(p_mid, drift, weather_mid, 1.0, true);
        let d_norm = clamp(d_raw / max(6.2 * cloud.params.x, 1e-3), 0.0, 1.0);
        return vec4<f32>(vec3<f32>(d_norm), 1.0);
    }

    let use_pathtrace = (cloud.shape.w >= 0.5 && cloud.shape.w < 1.5);

    if (use_pathtrace) {
        // ====================================================================
        // BACKEND B: Stochastic Volumetric Scattering Approximation
        // Candidate Real-Time Integrator (Zero Solari Dependency)
        // Evaluates single-scatter RTE with 1-sample Woodcock free-flight estimator
        // for higher-order multiple scattering; not an unbiased reference PT.
        // ====================================================================
        let dt = chord / f32(steps);
        let step_km = dt * radius_km;
        var rng_seed = u32(in.position.x) * 1973u + u32(in.position.y) * 9277u + u32(fract(dot(p0_deck, vec3<f32>(17.1, 31.7, 53.3))) * 100000.0);

        let dither_raw = dither2d(in.position.xy);
        let dither = (dither_raw - 0.5) * 0.70 + 0.5;
        for (var step = 0; step < CLOUD_MARCH_MAX; step += 1) {
            let t_sample = t0 + (f32(step) + dither) * dt;
            if (t_sample >= t1) {
                break;
            }

            let p_sample = ro + rd * t_sample;
            let f = clamp((t_sample - t0) / chord, 0.0, 1.0);
            let w2 = mix(w_entry, w_exit, f);
            let weather = vec3<f32>(w2.x, 0.0, w2.y);

            let d_sample = slab_density(p_sample, drift, weather, step_km, true);
            if (d_sample <= 0.001) {
                continue;
            }

            let step_tau = d_sample * ext * dt;
            let step_trans = exp(-step_tau);

            // 1. Next-Event Estimation towards star (micro-shadow tap for crisp crevices):
            let shadow_d0 = 180.0 * inv_radius * shadow_span;
            let shadow_d1 = 650.0 * inv_radius * shadow_span;
            let shadow_d2 = 1800.0 * inv_radius * shadow_span;
            let s0 = slab_density(p_sample + sun_dir * shadow_d0, drift, weather, 0.80, false);
            let s1 = slab_density(p_sample + sun_dir * shadow_d1, drift, weather, 2.20, false);
            let s2 = slab_density(p_sample + sun_dir * shadow_d2, drift, weather, 5.00, false);
            let sun_depth = (s0 * 0.65 + s1 * 0.90 + s2 * 1.50) * 2.8;

            let direct_atten = beer_powder(sun_depth);
            let up = p_sample / max(length(p_sample), 1e-6);
            let elev = dot(up, sun_dir);
            let day = smoothstep(-0.06, 0.16, elev) * sun_vis;
            let tint = mix(vec3<f32>(1.0, 0.45, 0.18), vec3<f32>(1.0, 0.98, 0.92), smoothstep(-0.02, 0.35, elev));

            let phase_direct = dual_lobe_phase(cos_theta);

            // 2. Secondary Woodcock Delta Tracking for Higher-Order Participating Medium Scatter:
            // Conservative extinction majorant: max procedural density is 6.2 * params.x,
            // so 6.5 * ext * params.x guarantees sigma_t(x) <= sigma_bar analytically.
            let max_majorant = max(ext * cloud.params.x * 6.5, 0.1);
            rng_seed = rng_seed * 1664525u + 1013904223u;
            let xi1 = max(f32((rng_seed >> 8u) & 0x00FFFFFFu) * (1.0 / 16777216.0), 1e-5);
            let dt_woodcock = -log(xi1) / max_majorant;

            // Phase-weighted bounce direction (biased forward with backward retro fill):
            let bounce_dir = normalize(rd * 0.65 + sun_dir * 0.35 + hash33(p_sample * 10.0) * 0.20);
            let p_bounce = p_sample + bounce_dir * dt_woodcock;
            let d_bounce = slab_density(p_bounce, drift, weather, dt_woodcock * radius_km, false);
            let bounce_accept = clamp(d_bounce * ext / max_majorant, 0.0, 1.0);

            let ms_eval = exp(-sun_depth * 0.28) * (0.35 + 0.65 * bounce_accept);

            let sun_scatter = direct_atten * (0.85 + 4.2 * phase_direct) + ms_eval * 1.15;
            let sun_radiance = tint * (day * sun_scatter * 2.8);

            let h01 = clamp((length(p_sample) - bot) / thick, 0.0, 1.0);
            let sky_amb = vec3<f32>(0.34, 0.50, 0.84) * (0.08 + 0.92 * day);
            let ground_amb = vec3<f32>(0.16, 0.20, 0.15) * (0.08 + 0.92 * day);
            let ambient_radiance = mix(ground_amb, sky_amb, h01 * 0.75 + 0.25) * 0.40;

            let in_scatter = sun_radiance + ambient_radiance;
            scat += transmittance * in_scatter * (1.0 - step_trans);
            transmittance *= step_trans;

            if (transmittance < 0.01) {
                transmittance = 0.0;
                break;
            }
        }
    } else {
        // ====================================================================
        // BACKEND A: Real-Time Stratified Ray Marching (bounded real-time budget on iGPU / Radeon 880M)
        // Midpoint numerical integration with space-skipping and Nyquist-bandlimited detail
        // ====================================================================
        let dt = chord / f32(steps);
        let step_km = dt * radius_km;
        let dither_raw = dither2d(in.position.xy);
        let dither = (dither_raw - 0.5) * 0.70 + 0.5;

        let is_diag_tau = (cloud.shape.w >= 2.5 && cloud.shape.w < 3.5);

        for (var i = 0; i < CLOUD_MARCH_MAX; i += 1) {
            let t_current = t0 + (f32(i) + dither) * dt;
            if (t_current >= t1) {
                break;
            }

            let p = ro + rd * t_current;
            let f = clamp((t_current - t0) / chord, 0.0, 1.0);
            let w2 = mix(w_entry, w_exit, f);
            let weather = vec3<f32>(w2.x, 0.0, w2.y);

            let density = slab_density(p, drift, weather, step_km, true);
            steps_taken += 1.0;
            if (density <= 0.001) {
                continue;
            }

            let step_ext = density * ext * dt;
            unsaturated_tau += step_ext;

            let h01 = clamp((length(p) - bot) / thick, 0.0, 1.0);

            // Sun self-shadowing: sample along sun direction with micro-shadow tap for crisp crevices
            let shadow_d0 = 180.0 * inv_radius * shadow_span;
            let shadow_d1 = 650.0 * inv_radius * shadow_span;
            let shadow_d2 = 1800.0 * inv_radius * shadow_span;

            let s0 = slab_density(p + sun_dir * shadow_d0, drift, weather, 0.80, false);
            let s1 = slab_density(p + sun_dir * shadow_d1, drift, weather, 2.20, false);
            let s2 = slab_density(p + sun_dir * shadow_d2, drift, weather, 5.00, false);

            let sun_depth = (s0 * 0.65 + s1 * 0.90 + s2 * 1.50) * 2.8;

            let direct_attenuation = beer_powder(sun_depth);
            let ms1 = exp(-sun_depth * 0.20) * 0.50;
            let ms2 = exp(-sun_depth * 0.05) * 0.28;

            let up = p / max(length(p), 1e-6);
            let elev = dot(up, sun_dir);
            let day = smoothstep(-0.06, 0.16, elev) * sun_vis;
            let tint = mix(vec3<f32>(1.0, 0.45, 0.18), vec3<f32>(1.0, 0.98, 0.92), smoothstep(-0.02, 0.35, elev));

            let phase_direct = dual_lobe_phase(cos_theta);
            let phase_bounce = 0.50 * hg_phase(0.35, cos_theta) + 0.50 * 0.079577;

            let sun_scatter = direct_attenuation * (0.85 + 4.2 * phase_direct) + ms1 * (0.50 + 1.6 * phase_bounce) + ms2 * 0.28;
            let sun_radiance = tint * (day * sun_scatter * 2.8);

            let sky_amb = vec3<f32>(0.34, 0.50, 0.84) * (0.08 + 0.92 * day);
            let ground_amb = vec3<f32>(0.16, 0.20, 0.15) * (0.08 + 0.92 * day);
            let height_occ = smoothstep(0.04, 0.40, h01);
            let core_occ = transmittance * transmittance;
            let dens_occ = clamp(1.0 - (density / max(6.2 * cloud.params.x, 1e-3)) * 0.35, 0.40, 1.0);
            let amb_occ = (0.30 + 0.70 * height_occ) * (0.20 + 0.80 * core_occ) * dens_occ;
            let ambient_radiance = mix(ground_amb, sky_amb, h01 * 0.75 + 0.25) * amb_occ * 0.55;

            let step_trans = exp(-step_ext);
            let point_light = sun_radiance + ambient_radiance;
            scat += transmittance * point_light * (1.0 - step_trans);
            transmittance *= step_trans;

            // In diagnostic Mode 3, bypass early-out to accumulate the entire unsaturated column optical depth:
            if (!is_diag_tau && transmittance < 0.01) {
                transmittance = 0.0;
                break;
            }
        }
    }

    let alpha = clamp(1.0 - transmittance, 0.0, 1.0);

    // Diagnostic Mode 3: Unsaturated Integrated Optical Depth (tau = sum sigma_ext * dt)
    if (cloud.shape.w >= 2.5 && cloud.shape.w < 3.5) {
        let norm_vis = clamp(unsaturated_tau / 16.0, 0.0, 1.0);
        return vec4<f32>(vec3<f32>(norm_vis), 1.0);
    }

    // Diagnostic Mode 4: Shading / In-Scattered Lighting
    if (cloud.shape.w >= 3.5 && cloud.shape.w < 4.5) {
        return vec4<f32>(scat, 1.0);
    }

    // Diagnostic Mode 5: Cloud-Only Mask
    if (cloud.shape.w >= 4.5 && cloud.shape.w < 5.5) {
        let mask = select(0.0, 1.0, alpha > 0.005);
        return vec4<f32>(vec3<f32>(mask), 1.0);
    }

    // Diagnostic Mode 6: March Step Count / Early-Out Heatmap
    if (cloud.shape.w >= 5.5 && cloud.shape.w < 6.5) {
        let step_ratio = clamp(steps_taken / f32(steps), 0.0, 1.0);
        return vec4<f32>(vec3<f32>(step_ratio), 1.0);
    }

    if (alpha < 0.001) {
        discard;
    }

    // Atmospheric aerial perspective with physically consistent premultiplied radiance:
    let r_cam_len = length(ro);
    let up_cam = select(vec3<f32>(0.0, 1.0, 0.0), ro / r_cam_len, r_cam_len > 1e-6);
    let elev_cam = dot(rd, up_cam);
    let march_dist_km = t0 * radius * 0.001;
    let air_trans = exp(-march_dist_km * 0.006);
    let haze_mix = clamp((1.0 - air_trans) * (1.0 - smoothstep(0.0, 0.35, elev_cam)), 0.0, 0.95);

    // Dynamic horizon haze color matched to sun elevation and day/night state:
    let up_sun = dot(up_cam, sun_dir);
    let day_sky = smoothstep(-0.06, 0.16, up_sun) * sun_vis;
    let sky_amb_horiz = vec3<f32>(0.34, 0.50, 0.84) * (0.08 + 0.92 * day_sky);
    let tint_horiz = mix(vec3<f32>(1.0, 0.45, 0.18), vec3<f32>(1.0, 0.98, 0.92), smoothstep(-0.02, 0.35, up_sun));
    let haze_color = mix(sky_amb_horiz * 1.5, tint_horiz * 0.8, pow(max(cos_theta, 0.0), 4.0) * 0.25);

    // In AlphaMode::Premultiplied:
    // Attenuate cloud source radiance by air transmittance and add in-scattered sky light proportional to alpha:
    let scat_atten = scat * (1.0 - haze_mix);
    let haze_inscatter = haze_color * (haze_mix * alpha);
    let out_rgb = scat_atten + haze_inscatter;

    return vec4<f32>(clamp(out_rgb, vec3<f32>(0.0), vec3<f32>(16.0)), alpha);
}
