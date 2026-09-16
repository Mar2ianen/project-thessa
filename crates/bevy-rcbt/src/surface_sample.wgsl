// Height decoding and geometry shared by both terrain consumers.
fn residual(page: vec4<u32>, sample_index: u32) -> i32 {
    let word = page_residuals[page.w + sample_index / 2u];
    let raw = (word >> ((sample_index & 1u) * 16u)) & 0xffffu;
    return select(i32(raw), i32(raw) - 65536, raw >= 32768u);
}

fn page_sample(page: vec4<u32>, u: f32, v: f32) -> f32 {
    if (page.z == 0u) {
        return 0.0;
    }
    let max_coord = f32(page.z - 1u);
    let px = clamp(u, 0.0, 1.0) * max_coord;
    let py = clamp(v, 0.0, 1.0) * max_coord;
    let x = u32(floor(px));
    let y = u32(floor(py));
    let x1 = min(x + 1u, page.z - 1u);
    let y1 = min(y + 1u, page.z - 1u);
    let tx = px - f32(x);
    let ty = py - f32(y);
    let grid = page.z;
    let h00 = bitcast<f32>(page.x) + f32(residual(page, y * grid + x)) * bitcast<f32>(page.y);
    let h10 = bitcast<f32>(page.x) + f32(residual(page, y * grid + x1)) * bitcast<f32>(page.y);
    let h01 = bitcast<f32>(page.x) + f32(residual(page, y1 * grid + x)) * bitcast<f32>(page.y);
    let h11 = bitcast<f32>(page.x) + f32(residual(page, y1 * grid + x1)) * bitcast<f32>(page.y);
    let top = h00 + (h10 - h00) * tx;
    let bottom = h01 + (h11 - h01) * tx;
    return top + (bottom - top) * ty;
}

struct CbtSurfaceSample {
    local_position: vec3<f32>,
    normal: vec3<f32>,
    height: f32,
};

// Shared by indexed compute and native mesh emission. Differentiate the
// unit cube sphere, never subtract two radius-sized f32 positions.
fn cbt_surface_sample(frame: CbtPrecisionFrame, page: vec4<u32>, uv: vec2<f32>) -> CbtSurfaceSample {
    let h = page_sample(page, uv.x, uv.y);
    let du = 1.0 / 32.0;
    let u0 = max(uv.x - du, 0.0);
    let u1 = min(uv.x + du, 1.0);
    let v0 = max(uv.y - du, 0.0);
    let v1 = min(uv.y + du, 1.0);
    let span = 2.0 * frame.radius_half_extent_len.y;
    let axis_u = frame.raw_axis_u.xyz;
    let axis_v = frame.raw_axis_v.xyz;
    let raw = frame.raw_center.xyz + span * (axis_u * (uv.x - 0.5) + axis_v * (uv.y - 0.5));
    let direction = normalize(raw);
    let metric = (frame.radius_half_extent_len.x + h) * span / length(raw);
    let dh_du = (page_sample(page, u1, uv.y) - page_sample(page, u0, uv.y)) / (u1 - u0);
    let dh_dv = (page_sample(page, uv.x, v1) - page_sample(page, uv.x, v0)) / (v1 - v0);
    let tangent_u = (axis_u - direction * dot(direction, axis_u)) * metric + direction * dh_du;
    let tangent_v = (axis_v - direction * dot(direction, axis_v)) * metric + direction * dh_dv;
    return CbtSurfaceSample(cbt_local_offset(frame, uv, h), normalize(cross(tangent_u, tangent_v)), h);
}
