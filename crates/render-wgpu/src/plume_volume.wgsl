struct Draw {
    rect: vec4<f32>,
    origin_len: vec4<f32>,
    axis_radius: vec4<f32>,
    // station count, steps, time, erosion
    controls: vec4<f32>,
    // advection speed, diamonds, tail dissolve, reserved
    flow: vec4<f32>,
    radial_reference: vec4<f32>,
    shock_cells: vec4<f32>,
}
struct Station { geometry: vec4<f32>, emission: vec4<f32>, }
@group(1) @binding(0) var<uniform> draw: Draw;
@group(1) @binding(1) var<storage, read> stations: array<Station>;

@vertex fn vertex(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let corners = array<vec2<f32>, 6>(vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0),
        vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0));
    var rect = draw.rect;
    if draw.radial_reference.w > 1.0 {
        // Include pixels whose footprint overlaps the conservative support,
        // even when their centre lies outside its projected rectangle.
        let pad = 2.0 / vec2<f32>(textureDimensions(scene_depth));
        rect = vec4(rect.xy-pad,rect.zw+pad);
    }
    return vec4<f32>(mix(rect.xy, rect.zw, corners[index]), 0.0, 1.0);
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

// Shortest-arc transport of the nozzle frame; no world-up pole switch.
fn transported_radial_reference(axis: vec3<f32>) -> vec3<f32> {
    let radial = draw.radial_reference.xyz;
    let start = draw.axis_radius.xyz;
    let v = cross(start,axis);
    let denominator = 1.0+dot(start,axis);
    if denominator < 1e-5 { return normalize(radial-axis*dot(radial,axis)); }
    return normalize(radial+cross(v,radial)+cross(v,cross(v,radial))/denominator);
}

fn trace_straight(pixel: vec2<f32>) -> vec4<f32> {
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
        if c > 0.0 { return vec4(0.0); }
    } else {
        let disc = b * b - a * c;
        if disc <= 0.0 { return vec4(0.0); }
        let root = sqrt(disc);
        t0 = max(t0, (-b - root) / a);
        t1 = min(t1, (-b + root) / a);
    }
    if abs(da) < 1e-7 {
        if oa < 0.0 || oa > len { return vec4(0.0); }
    } else {
        let ta = -oa / da; let tb = (len - oa) / da;
        t0 = max(t0, min(ta, tb)); t1 = min(t1, max(ta, tb));
    }
    // Integrate only up to the opaque scene intersection, even when an
    // opaque object lies inside the field. No proxy-plane depth shortcut.
    let depth = textureLoad(scene_depth, vec2<i32>(pixel.xy), 0);
    if depth < 1.0 { t1 = min(t1, dot(unproject(pixel.xy, depth) - ro, rd)); }
    if t1 <= t0 { return vec4(0.0); }
    var steps = u32(clamp(draw.controls.y, 1.0, 16.0));
    // Warp the fixed sample budget toward the closest approach to the jet
    // axis. Uniform steps over the far-tail bound miss thin local slices of
    // a long vacuum barrel. Every bin still spans its actual physical dt.
    var closest = (t0 + t1) * 0.5;
    if a > 1e-8 { closest = clamp(-b / a, t0, t1); }
    let z_closest = clamp(oa + da * closest, 0.0, len);
    let closest_point = oc+rd*closest;
    let core_offset = closest_point-axis*clamp(dot(closest_point,axis),0.0,len);
    let refine = draw.flow.y > 0.0 && draw.shock_cells.x > 0.0 && draw.shock_cells.z > 1.0
        && dot(core_offset,core_offset) < 1.44*draw.shock_cells.y*draw.shock_cells.y;
    var warp_radius = max(sample_station(z_closest).geometry.y * 2.0, 1e-4);
    if refine {
        // Midpoint error scales as amplitude/N^2. Collisional
        // contrast scales as C^2, so N scales with compression C.
        let radial_budget = u32(clamp(ceil(draw.controls.y*draw.shock_cells.z),1.0,64.0));
        steps = max(radial_budget,u32(clamp(ceil(abs(da)*(t1-t0)/draw.shock_cells.x*8.0),1.0,64.0)));
        warp_radius = min(warp_radius,draw.shock_cells.y);
    }
    var warp = a > 1e-8;
    if refine && a < 0.05 {
        closest = t0;
        warp_radius = max(draw.shock_cells.x,1e-4);
        warp = true;
    }
    let angle0 = atan((t0 - closest) / warp_radius);
    let angle1 = atan((t1 - closest) / warp_radius);
    var transmission = 1.0; var rgb = vec3<f32>(0.0);
    for (var i = 0u; i < steps; i += 1u) {
        var begin = mix(t0, t1, f32(i) / f32(steps));
        var end = mix(t0, t1, f32(i + 1u) / f32(steps));
        if warp {
            begin = closest + warp_radius * tan(mix(angle0, angle1, f32(i) / f32(steps)));
            end = closest + warp_radius * tan(mix(angle0, angle1, f32(i + 1u) / f32(steps)));
        }
        let dt = end - begin;
        let p = oc + rd * ((begin + end) * 0.5);
        let z = clamp(dot(p, axis), 0.0, len);
        let station = sample_station(z);
        let radial = p - axis * z;
        let radius = max(station.geometry.y, 1e-6);
        let core_r = length(radial);
        var r = core_r;
        if draw.controls.w > 0.0 {
            let u1 = draw.radial_reference.xyz; let u2 = cross(axis, u1);
            // Independent zero-mean modal shell representation; no copied
            // game-owned optical/noise bake. Wavelength follows local radius.
            let az = atan2(dot(radial, u2), dot(radial, u1) + 1e-6);
            let phase = (z - draw.controls.z * draw.flow.x) / (4.0 * radius);
            let n = (sin(phase + 3.0 * az) + sin(phase * 1.7 - 5.0 * az)) * 0.25;
            r += n * draw.controls.w * radius * smoothstep(0.1, 0.9, r / radius);
        }
        let u = abs(r) / radius;
        let core_u = core_r / radius;
        var weight = exp(-2.2 * core_u * core_u) + 0.18 * exp(-0.7 * u * u);
        // No hard radial skirt clip or fitted mean extinction. Optical
        // integration is the CPU sampler over the same bounded chord.
        if draw.flow.z > 0.0 { weight *= 1.0 - smoothstep(0.70, 1.0, z / len); }
        var shock = 1.0;
        if draw.flow.y > 0.0 { shock = shock_multiplier(station.geometry.w, core_u); }
        var cells = vec2(1.0);
        if draw.flow.y > 0.0 { cells = spatial_shock(z,core_r,draw.shock_cells); }
        let extinction = max(station.geometry.z * weight * shock * cells.x, 0.0);
        let tau = extinction * dt;
        let absorb = exp(-tau);
        // Integral exp(-sigma*s) ds. The thin-cell series avoids cancellation;
        // its omitted relative term is <= 4.2e-8 below tau=0.01, before f32 error.
        var emission_length = dt * (1.0 - tau * (0.5 - tau / 6.0));
        if tau >= 0.01 { emission_length = (1.0 - absorb) / extinction; }
        rgb += transmission * max(station.emission.rgb * weight * shock * cells.y, vec3(0.0)) * emission_length;
        transmission *= absorb;
    }
    return vec4<f32>(rgb * frame.exposure.x, clamp(1.0 - transmission, 0.0, 1.0));
}

// Finite sensor footprint. Average transported radiance and opacity, never
// medium coefficients before the nonlinear Beer-Lambert integration.
fn pixel_span(pixel: vec2<f32>, distance: f32) -> f32 {
    let ro = frame.camera.xyz;
    let rd = normalize(unproject(pixel,0.5)-ro);
    let dx = normalize(unproject(pixel+vec2(1.0,0.0),0.5)-ro)-rd;
    let dy = normalize(unproject(pixel+vec2(0.0,1.0),0.5)-ro)-rd;
    return max(length(dx),length(dy))*distance;
}

fn pixel_grid(pixel: vec2<f32>, point: vec3<f32>, z: f32) -> u32 {
    let span = pixel_span(pixel,length(point-frame.camera.xyz));
    let rd = normalize(unproject(pixel,0.5)-frame.camera.xyz);
    let offset = point-frame.camera.xyz;
    let separation = length(offset-rd*dot(offset,rd));
    let radius = max(sample_station(z).geometry.y,1e-6);
    // Beyond 4R the Gaussian skirt is below 2.5e-6 of its axial weight.
    // Keep point transport there; do not supersample a distant narrow
    // segment merely because a broad conservative capsule covers this ray.
    let shell_margin = 0.5*max(draw.controls.w,0.0);
    if separation > (4.0+shell_margin)*radius+span { return 1u; }
    var feature = radius*0.5;
    // Spend the fine grid on sharp compression sheets. The smooth skirt
    // uses at most four rays, including in a long curved vacuum barrel.
    var budget = min(2u,u32(draw.radial_reference.w));
    if draw.flow.y > 0.0 && draw.shock_cells.x > 0.0 && draw.shock_cells.z > 1.0
        && separation < 1.2*draw.shock_cells.y+span {
        feature = min(feature,0.08*draw.shock_cells.y);
        budget = u32(draw.radial_reference.w);
    }
    let demand = span/max(feature,1e-6);
    var grid = 1u;
    if demand > 1.0 { grid = 2u; }
    if demand > 2.0 { grid = 4u; }
    if demand > 4.0 { grid = 8u; }
    return min(grid,budget);
}

@fragment fn fragment(@builtin(position) pixel: vec4<f32>) -> @location(0) vec4<f32> {
    var grid = 1u;
    if draw.radial_reference.w > 1.0 {
        let ro = frame.camera.xyz;
        let rd = normalize(unproject(pixel.xy,0.5)-ro);
        let ab = draw.axis_radius.xyz*draw.origin_len.w;
        let offset = draw.origin_len.xyz-ro;
        let lateral = ab-rd*dot(ab,rd);
        let fraction = clamp(-dot(offset-rd*dot(offset,rd),lateral)/max(dot(lateral,lateral),1e-12),0.0,1.0);
        grid = pixel_grid(pixel.xy,draw.origin_len.xyz+ab*fraction,draw.origin_len.w*fraction);
    }
    var sum = vec4(0.0);
    for (var y = 0u; y < grid; y += 1u) {
        for (var x = 0u; x < grid; x += 1u) {
            let offset = (vec2<f32>(f32(x),f32(y))+0.5)/f32(grid)-0.5;
            sum += trace_straight(pixel.xy+offset);
        }
    }
    return sum/f32(grid*grid);
}
