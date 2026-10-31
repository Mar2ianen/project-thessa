// The same participating medium swept through a camera-independent 3D path.
@group(1) @binding(2) var<storage, read> centers: array<vec4<f32>>;

struct CurveLocation { radial: vec3<f32>, z: f32, axis: vec3<f32>, valid: bool, metric: f32, }
fn curve_location(p: vec3<f32>) -> CurveLocation {
    var best = 1e30;
    var result = CurveLocation(vec3(0.0), 0.0, vec3(0.0, 0.0, 1.0), false, 1.0);
    let count = u32(draw.flow.w);
    for (var j = 0u; j + 1u < count; j += 1u) {
        let a = centers[j]; let b = centers[j + 1u];
        let ab = b.xyz - a.xyz;
        let t = dot(p - a.xyz, ab) / dot(ab, ab);
        // Flat inlet/outlet. Rounded internal joins close the swept tube.
        if (j == 0u && t < 0.0) || (j + 2u == count && t > 1.0) { continue; }
        let fraction = clamp(t, 0.0, 1.0);
        let radial = p - mix(a.xyz, b.xyz, fraction);
        let d2 = dot(radial, radial);
        if d2 < best {
            best = d2;
            result = CurveLocation(radial, mix(a.w, b.w, fraction), ab, true, b.w-a.w);
        }
    }
    result.metric /= length(result.axis);
    result.axis = normalize(result.axis);
    result.valid = result.valid && best <= draw.axis_radius.w * draw.axis_radius.w;
    return result;
}

fn curve_sphere_interval(ro: vec3<f32>, rd: vec3<f32>, center: vec3<f32>, radius: f32) -> vec2<f32> {
    let oc = ro-center; let b = dot(oc,rd);
    let perpendicular = oc-rd*b;
    let discriminant = radius*radius-dot(perpendicular,perpendicular);
    if discriminant <= 0.0 { return vec2(1.0,-1.0); }
    let root = sqrt(discriminant);
    return vec2(max(0.0,-b-root),-b+root);
}

// Rotation-invariant swept support. World-axis boxes may bound coverage, but
// their orientation-dependent ray endpoints must not move quadrature bins.
fn curve_capsule_interval(ro: vec3<f32>, rd: vec3<f32>, a: vec3<f32>, b: vec3<f32>, radius: f32) -> vec2<f32> {
    let ab = b-a; let len = length(ab); let axis = ab/len;
    let oc = ro-a; let da = dot(rd,axis); let oa = dot(oc,axis);
    let lateral_ray = rd-axis*da; let lateral_origin = oc-axis*oa;
    let qa = dot(lateral_ray,lateral_ray); let qb = dot(lateral_ray,lateral_origin);
    let qc = dot(lateral_origin,lateral_origin)-radius*radius;
    var near = 0.0; var far = 1e20;
    var valid = true;
    if qa < 1e-12 {
        valid = qc <= 0.0;
    } else {
        let discriminant = qb*qb-qa*qc;
        if discriminant <= 0.0 { valid = false; }
        else {
            let root = sqrt(discriminant);
            near = max(near,(-qb-root)/qa); far = min(far,(-qb+root)/qa);
        }
    }
    if abs(da) < 1e-7 { valid = valid && oa >= 0.0 && oa <= len; }
    else {
        let ta = -oa/da; let tb = (len-oa)/da;
        near = max(near,min(ta,tb)); far = min(far,max(ta,tb));
    }
    if !valid || far <= near { near = 1e20; far = -1e20; }
    let first = curve_sphere_interval(ro,rd,a,radius);
    if first.y > first.x { near = min(near,first.x); far = max(far,first.y); }
    let last = curve_sphere_interval(ro,rd,b,radius);
    if last.y > last.x { near = min(near,last.x); far = max(far,last.y); }
    return vec2(near,far);
}

@fragment fn curved_fragment(@builtin(position) pixel: vec4<f32>) -> @location(0) vec4<f32> {
    let ro = frame.camera.xyz;
    let rd = normalize(unproject(pixel.xy, 0.5) - ro);
    let depth = textureLoad(scene_depth, vec2<i32>(pixel.xy), 0);
    var opaque = 1e20;
    if depth < 1.0 { opaque = dot(unproject(pixel.xy, depth) - ro, rd); }
    let count = u32(draw.flow.w);
    let radius = draw.axis_radius.w;
    var intervals: array<vec2<f32>, 32>;
    var used = 0u;
    // Sort and merge conservative segment intervals before integration:
    // intersections are visited once, including disconnected ray crossings.
    for (var j = 0u; j + 1u < count; j += 1u) {
        let a = centers[j].xyz; let b = centers[j + 1u].xyz;
        var hit = curve_capsule_interval(ro, rd, a, b, radius);
        hit.y = min(hit.y, opaque);
        if hit.y <= hit.x { continue; }
        var slot = used;
        while slot > 0u {
            if intervals[slot - 1u].x <= hit.x { break; }
            intervals[slot] = intervals[slot - 1u]; slot -= 1u;
        }
        intervals[slot] = hit; used += 1u;
    }
    if used == 0u { discard; }
    var merged = 1u;
    for (var j = 1u; j < used; j += 1u) {
        if intervals[j].x <= intervals[merged - 1u].y {
            intervals[merged - 1u].y = max(intervals[merged - 1u].y, intervals[j].y);
        } else { intervals[merged] = intervals[j]; merged += 1u; }
    }
    let steps = u32(clamp(draw.controls.y, 1.0, 16.0));
    var transmission = 1.0; var rgb = vec3(0.0);
    for (var interval = 0u; interval < merged; interval += 1u) {
        let t0 = intervals[interval].x; let t1 = intervals[interval].y;
        var closest = (t0 + t1) * 0.5;
        var nearest2 = 1e30; var closest_z = 0.0;
        // Closest point between the view ray and each finite center segment.
        for (var j = 0u; j + 1u < count; j += 1u) {
            let a = centers[j]; let b = centers[j+1u]; let ab = b.xyz-a.xyz;
            let lateral = ab - rd * dot(ab,rd);
            let offset = a.xyz-ro; let offset_lateral = offset-rd*dot(offset,rd);
            let fraction = clamp(-dot(offset_lateral,lateral)/max(dot(lateral,lateral),1e-12),0.0,1.0);
            let point = mix(a.xyz,b.xyz,fraction);
            let t = clamp(dot(point-ro,rd),t0,t1);
            let separation = point-(ro+rd*t); let d2 = dot(separation,separation);
            if d2 < nearest2 {
                nearest2 = d2; closest = t;
                closest_z = mix(a.w,b.w,fraction);
            }
        }
        let warp_radius = max(2.0*sample_station(closest_z).geometry.y,1e-4);
        let angle0 = atan((t0-closest)/warp_radius);
        let angle1 = atan((t1-closest)/warp_radius);
        for (var i = 0u; i < steps; i += 1u) {
            let begin = closest+warp_radius*tan(mix(angle0,angle1,f32(i)/f32(steps)));
            let end = closest+warp_radius*tan(mix(angle0,angle1,f32(i+1u)/f32(steps)));
            let dt = end-begin;
            let local = curve_location(ro+rd*((begin+end)*0.5));
            if !local.valid { continue; }
            let station = sample_station(local.z);
            let local_radius = max(station.geometry.y,1e-6);
            var r = length(local.radial);
            if draw.controls.w > 0.0 {
                let u1 = transported_radial_reference(local.axis); let u2 = cross(local.axis,u1);
                let az = atan2(dot(local.radial,u2),dot(local.radial,u1)+1e-6);
                let phase = (local.z-draw.controls.z*draw.flow.x)/(4.0*local_radius);
                let n = (sin(phase+3.0*az)+sin(phase*1.7-5.0*az))*0.25;
                r += n*draw.controls.w*local_radius*smoothstep(0.1,0.9,r/local_radius);
            }
            let u = abs(r)/local_radius;
            var weight = (exp(-2.2*u*u)+0.18*exp(-0.7*u*u))*local.metric;
            if draw.flow.z > 0.0 { weight *= 1.0-smoothstep(0.70,1.0,local.z/draw.origin_len.w); }
            var shock = 1.0;
            if draw.flow.y > 0.0 { shock = shock_multiplier(station.geometry.w,u); }
            let extinction = max(station.geometry.z*weight*shock,0.0);
            let tau = extinction*dt; let absorb = exp(-tau);
            var emission_length = dt*(1.0-tau*(0.5-tau/6.0));
            if tau >= 0.01 { emission_length = (1.0-absorb)/extinction; }
            rgb += transmission*max(station.emission.rgb*weight*shock,vec3(0.0))*emission_length;
            transmission *= absorb;
        }
    }
    return vec4(rgb*frame.exposure.x,clamp(1.0-transmission,0.0,1.0));
}
