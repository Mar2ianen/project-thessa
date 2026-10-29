struct Draw {
    rect: vec4<f32>,
    origin_len: vec4<f32>,
    axis_radius: vec4<f32>,
    // station count, steps, time, erosion
    controls: vec4<f32>,
    // advection speed, diamonds, tail dissolve, reserved
    flow: vec4<f32>,
}
struct Station { geometry: vec4<f32>, emission: vec4<f32>, }
@group(1) @binding(0) var<uniform> draw: Draw;
@group(1) @binding(1) var<storage, read> stations: array<Station>;

@vertex fn vertex(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let corners = array<vec2<f32>, 6>(vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0),
        vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0));
    return vec4<f32>(mix(draw.rect.xy, draw.rect.zw, corners[index]), 0.0, 1.0);
}

fn sample_station(z: f32) -> Station {
    let count = u32(draw.controls.x);
    var lo = 0u;
    for (var i = 1u; i < count - 1u; i += 1u) {
        if stations[i].geometry.x >= z { break; }
        lo = i;
    }
    let a = stations[lo];
    let b = stations[lo + 1u];
    let t = clamp((z - a.geometry.x) / max(b.geometry.x - a.geometry.x, 1e-9), 0.0, 1.0);
    return Station(mix(a.geometry, b.geometry, t), mix(a.emission, b.emission, t));
}

@fragment fn fragment(@builtin(position) pixel: vec4<f32>) -> @location(0) vec4<f32> {
    let ro = frame.camera.xyz;
    let rd = normalize(unproject(pixel.xy, 0.5) - ro);
    let axis = draw.axis_radius.xyz;
    let oc = ro - draw.origin_len.xyz;
    let len = draw.origin_len.w;
    let rmax = draw.axis_radius.w;
    let da = dot(rd, axis); let oa = dot(oc, axis);
    // Form perpendicular components before squaring (avoids cancellation
    // of large axial distances and the undefined pow(negative, 2) case).
    let lateral_ray = rd - axis * da;
    let lateral_origin = oc - axis * oa;
    let a = dot(lateral_ray, lateral_ray);
    let b = dot(lateral_origin, lateral_ray);
    let c = dot(lateral_origin, lateral_origin) - rmax * rmax;
    var t0 = 0.0; var t1 = 1e20;
    if a < 1e-12 {
        if c > 0.0 { discard; }
    } else {
        let disc = b * b - a * c;
        if disc <= 0.0 { discard; }
        let root = sqrt(disc);
        t0 = max(t0, (-b - root) / a);
        t1 = min(t1, (-b + root) / a);
    }
    if abs(da) < 1e-7 {
        if oa < 0.0 || oa > len { discard; }
    } else {
        let ta = -oa / da; let tb = (len - oa) / da;
        t0 = max(t0, min(ta, tb)); t1 = min(t1, max(ta, tb));
    }
    // Integrate only up to the opaque scene intersection, even when an
    // opaque object lies inside the field. No proxy-plane depth shortcut.
    let depth = textureLoad(scene_depth, vec2<i32>(pixel.xy), 0);
    if depth < 1.0 { t1 = min(t1, dot(unproject(pixel.xy, depth) - ro, rd)); }
    if t1 <= t0 { discard; }
    let steps = u32(clamp(draw.controls.y, 1.0, 16.0));
    // Warp the fixed sample budget toward the closest approach to the jet
    // axis. Uniform steps over the far-tail bound miss thin local slices of
    // a long vacuum barrel. Every bin still spans its actual physical dt.
    var closest = (t0 + t1) * 0.5;
    if a > 1e-8 { closest = clamp(-b / a, t0, t1); }
    let z_closest = clamp(oa + da * closest, 0.0, len);
    let warp_radius = max(sample_station(z_closest).geometry.y * 2.0, 1e-4);
    let angle0 = atan((t0 - closest) / warp_radius);
    let angle1 = atan((t1 - closest) / warp_radius);
    var transmission = 1.0; var rgb = vec3<f32>(0.0);
    for (var i = 0u; i < steps; i += 1u) {
        var begin = mix(t0, t1, f32(i) / f32(steps));
        var end = mix(t0, t1, f32(i + 1u) / f32(steps));
        if a > 1e-8 {
            begin = closest + warp_radius * tan(mix(angle0, angle1, f32(i) / f32(steps)));
            end = closest + warp_radius * tan(mix(angle0, angle1, f32(i + 1u) / f32(steps)));
        }
        let dt = end - begin;
        let p = oc + rd * ((begin + end) * 0.5);
        let z = clamp(dot(p, axis), 0.0, len);
        let station = sample_station(z);
        let radial = p - axis * z;
        let radius = max(station.geometry.y, 1e-6);
        var r = length(radial);
        if draw.controls.w > 0.0 {
            let reference = select(vec3(0.0, 1.0, 0.0), vec3(1.0, 0.0, 0.0), abs(axis.y) > 0.9);
            let u1 = normalize(cross(axis, reference)); let u2 = cross(axis, u1);
            // Independent zero-mean modal shell representation; no copied
            // game-owned optical/noise bake. Wavelength follows local radius.
            let az = atan2(dot(radial, u2), dot(radial, u1) + 1e-6);
            let phase = (z - draw.controls.z * draw.flow.x) / (4.0 * radius);
            let n = (sin(phase + 3.0 * az) + sin(phase * 1.7 - 5.0 * az)) * 0.25;
            r += n * draw.controls.w * radius * smoothstep(0.1, 0.9, r / radius);
        }
        let u = min(abs(r) / radius, 3.0);
        var weight = exp(-2.2 * u * u) + 0.18 * exp(-0.7 * u * u);
        // No hard radial skirt clip or fitted mean extinction. Optical
        // integration is the CPU sampler over the same bounded chord.
        if draw.flow.z > 0.0 { weight *= 1.0 - smoothstep(0.70, 1.0, z / len); }
        let shock = max(mix(1.0, station.geometry.w, draw.flow.y), 0.15);
        let absorb = exp(-max(station.geometry.z * weight * shock, 0.0) * dt);
        rgb += transmission * max(station.emission.rgb * weight * shock, vec3(0.0)) * dt * absorb;
        transmission *= absorb;
    }
    return vec4<f32>(rgb * frame.exposure.x, clamp(1.0 - transmission, 0.0, 1.0));
}
