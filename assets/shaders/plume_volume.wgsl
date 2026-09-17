// Plume analytic-volume march (`docs/38` sections 6, 8).
//
// A camera-facing ribbon (cylindrical billboard) is COVERAGE ONLY: the shape
// comes from the field below, never from the mesh. Each pixel marches a view
// ray through the round cross-section and Beer-Lambert-integrates extinction
// plus emission. Math mirrors `plume-core/src/medium.rs` (radial weights,
// shock modulation, axial decay); the CPU `integrate_ray` is the oracle and
// this pass must agree with it within representation error.
//
// AxialRadius/decay note: the CPU profile is the single source of truth;
// uniforms carry its fitted params (`mean_radius`, `ramp_rgb`, axial decay
// `1/(1+6zn^2)`). The CPU `integrate_ray` is the oracle; representation
// detail (noise deformation, near fade) stays inside tolerance.

#import bevy_pbr::{
    mesh_view_bindings::view,
    forward_io::VertexOutput,
}

struct PlumeUniforms {
    // origin.xyz + length (m)
    origin_len: vec4<f32>,
    // exhaust axis (unit, world) + exit radius (m)
    axis_r0: vec4<f32>,
    // tail radius, shock spacing, shock amplitude, time (s)
    shape_time: vec4<f32>,
    // luminosity gain, mean extinction (1/m, field-derived), step count,
    // advection speed (m/s)
    march: vec4<f32>,
    // core rgb + edge-erosion strength
    core_rgb: vec4<f32>,
    mid_rgb: vec4<f32>,
    edge_rgb: vec4<f32>,
}

// Material bind group is 3 in Bevy 0.19 (MATERIAL_BIND_GROUP_INDEX), NOT 2:
// group 0 = view, 1 = mesh, 2 = lights/clustered, 3 = material.
@group(3) @binding(0)
var<uniform> plume: PlumeUniforms;

fn hash12(p: vec2<f32>) -> f32 {
    var p3 = fract(vec3<f32>(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

fn vnoise(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let a = hash12(i);
    let b = hash12(i + vec2<f32>(1.0, 0.0));
    let c = hash12(i + vec2<f32>(0.0, 1.0));
    let d = hash12(i + vec2<f32>(1.0, 1.0));
    return a + (b - a) * u.x + (c - a) * u.y + (a - b - c + d) * u.x * u.y;
}

// Same steep core falloff as plume-core `ramp_rgb`: white-blue confined
// to the first diameters, then violet/pink, then transparent edge.
fn axial_ramp(zn: f32, core_rgb: vec3<f32>, mid_rgb: vec3<f32>, edge_rgb: vec3<f32>) -> vec3<f32> {
    let core_bias = exp(-6.0 * zn);
    let edge_bias = 1.0 - exp(-2.0 * zn);
    let col = core_rgb * core_bias + mid_rgb * (1.0 - core_bias);
    let edge_mix = min(edge_bias * 0.45, 0.6);
    return col * (1.0 - edge_mix) + edge_rgb * edge_mix;
}

// Same axial decay as the CPU builder: 1 / (1 + 6 z_n^2).
fn axial_decay(zn: f32) -> f32 {
    return 1.0 / (1.0 + 6.0 * zn * zn);
}

// Same mean radius as plume-core `mean_radius`: linear spread plus the
// saturating near-lip fan bulge; R(0) == r0 exactly.
fn mean_radius(r0: f32, spread: f32, fan: f32, z: f32) -> f32 {
    let d = max(2.0 * r0, 1e-6);
    let zc = max(z, 0.0);
    return r0 * (1.0 + spread * zc / d) + r0 * fan * (1.0 - exp(-zc / (2.0 * d)));
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let origin = plume.origin_len.xyz;
    let len = max(plume.origin_len.w, 1e-3);
    let axis = plume.axis_r0.xyz;
    let r0 = plume.axis_r0.w;
    let r1 = plume.shape_time.x;
    let spacing = max(plume.shape_time.y, 1e-3);
    let amp = plume.shape_time.z;
    let time = plume.shape_time.w;
    let gain = plume.march.x;
    let ext_mean = plume.march.y;
    let steps = max(i32(plume.march.z + 0.5), 1);
    let advect = plume.march.w;
    let erosion = plume.core_rgb.w;

    // View ray in world space.
    let ro = view.world_position.xyz;
    let frag_pos = in.world_position.xyz;
    let rd = normalize(frag_pos - ro);

    // Intersect the bounding cylinder (axis segment [0, len], radius rmax).
    let rmax = max(r0, r1) * 1.6;
    let oc = ro - origin;
    let a = dot(rd, rd) - pow(dot(rd, axis), 2.0);
    let b = dot(oc, rd) - dot(oc, axis) * dot(rd, axis);
    let c = dot(oc, oc) - pow(dot(oc, axis), 2.0) - rmax * rmax;
    let disc = b * b - a * c;
    if (disc <= 0.0 || a <= 1e-9) {
        discard;
    }
    let sq = sqrt(disc);
    var t0 = max((-b - sq) / a, 0.0);
    var t1 = (-b + sq) / a;
    // Clip to the axial slab [0, len].
    let z0 = dot(oc, axis);
    let dz = dot(rd, axis);
    if (abs(dz) > 1e-6) {
        t0 = max(t0, min((0.0 - z0) / dz, (len - z0) / dz));
        t1 = min(t1, max((0.0 - z0) / dz, (len - z0) / dz));
    }
    if (t1 <= t0) {
        discard;
    }

    let steps_f = f32(steps);
    let dt = (t1 - t0) / steps_f;
    // Camera-inside fade: marching the full bound with the eye inside the
    // plume whites out the frame (vacuum barrels are huge). Fade out as the
    // eye penetrates the bound cylinder; outside it stays 1.
    let z_cam = dot(oc, axis);
    let d_lat = length(oc - axis * z_cam);
    let axial_in = step(0.0, z_cam) * step(z_cam, len);
    let penetration = axial_in * clamp(1.0 - d_lat / max(rmax, 1e-3), 0.0, 1.0);
    let near_fade = 1.0 - 0.92 * smoothstep(0.0, 0.55, penetration);
    var transmittance = 1.0;
    var rgb = vec3<f32>(0.0);
    let spread = plume.mid_rgb.w;
    let fan = plume.edge_rgb.w;
    for (var i = 0; i < 16; i += 1) {
        if (i >= steps) {
            break;
        }
        let t = t0 + (f32(i) + 0.5) * dt;
        let p = ro + rd * t - origin;
        let z = clamp(dot(p, axis), 0.0, len);
        let zn = z / len;
        let radius = mean_radius(r0, spread, fan, z);
        // Turbulence deforms the SHELL (radius), not just opacity: advected
        // noise offsets the sample radius before the weight is evaluated,
        // so edges wobble instead of merely flickering.
        let n = vnoise(vec2<f32>(z * 2.6 - time * advect, zn * 9.0));
        let n2 = vnoise(vec2<f32>(z * 5.8 - time * advect * 1.7, 4.2 + zn * 13.0));
        let deform = (n * 0.65 + n2 * 0.35 - 0.5) * erosion * radius;
        let radial = p - axis * z;
        let r = length(radial) + deform * smoothstep(0.1, 0.9, length(radial) / max(radius, 1e-4));
        let u = min(r / max(radius, 1e-4), 3.0);
        // Radial weight mirrors medium.rs exactly.
        var w = exp(-2.2 * u * u) + 0.18 * exp(-0.7 * u * u);
        // Residual alpha erosion on top of the geometric deformation.
        let turb = (n * 0.65 + n2 * 0.35 - 0.5) * erosion * smoothstep(0.35, 1.0, u);
        w = max(w * (1.0 - turb), 0.0);
        // Peaked shock cells (not arcade-perfect diamonds, but periodic
        // compression): pow-shaping concentrates the modulation into bands
        // so supersonic structure reads through the march.
        let span = max(spacing, len * 0.05);
        let cell = pow(0.5 + 0.5 * cos(6.2831853 * z / spacing), 1.5);
        let shock = 1.0 + amp * 0.75 * (2.0 * cell - 1.0)
            * exp(-z / (3.0 * span));
        let col = axial_ramp(zn, plume.core_rgb.rgb, plume.mid_rgb.rgb, plume.edge_rgb.rgb);
        let decay = max(axial_decay(zn), 0.02);
        let emission = col * (gain * decay * max(w * max(shock, 0.15), 0.0));
        // Extinction mirrors the CPU oracle (station mean x weight x shock).
        let ext = max(ext_mean * w * max(shock, 0.15), 0.0);
        let absorb = exp(-ext * dt);
        rgb += transmittance * emission * dt * absorb;
        transmittance *= absorb;
    }

    let alpha = clamp(1.0 - transmittance, 0.0, 1.0);
    // Premultiplied-style HDR output for the Add blend path.
    return vec4<f32>(rgb * near_fade, alpha * near_fade);
}
