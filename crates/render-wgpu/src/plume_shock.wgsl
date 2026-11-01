// First circular-jet pressure mode, matching plume-core::shock_multiplier.
// Degree-six J0 polynomial has <5.3e-7 absolute truncation error on the core.
fn shock_multiplier(axial: f32, u: f32) -> f32 {
    if u >= 1.0 || axial == 1.0 { return 1.0; }
    let q = 1.4457964907366962 * u * u;
    let mode = 1.0 + q * (-1.0 + q * (0.25 + q * (-1.0 / 36.0
        + q * (1.0 / 576.0 + q * (-1.0 / 14400.0 + q / 518400.0)))));
    return max(1.0 + (max(axial, 0.15) - 1.0) * clamp(mode, 0.0, 1.0), 0.15);
}

// Independent spatial double-cone closure; matches plume-core::shock.
fn spatial_shock(z: f32, r: f32, cell: vec4<f32>) -> vec2<f32> {
    if cell.x <= 0.0 || cell.z <= 1.0 { return vec2(1.0); }
    let x = max(z,0.0)/cell.x;
    let u = abs(r)/cell.y;
    if u >= 1.2 { return vec2(1.0); }
    let phase = fract(x+cell.w);
    let width = 1.0-abs(2.0*phase-1.0);
    let softness = 0.08+0.04*x;
    let section = max(width,softness);
    let axial = 2.0*phase-1.0;
    let body = exp(-2.0*((u/section)*(u/section)+axial*axial));
    let distance = (u-abs(axial))/softness;
    let incident = exp(-0.5*distance*distance);
    let compression_region = max(body,incident);
    let mask = 1.0-smoothstep(0.8,1.2,u);
    let coherence = exp(-x/3.0)*(1.0-exp(-max(z,0.0)/cell.y));
    let density = 1.0/cell.z+(cell.z-1.0/cell.z)*compression_region;
    return vec2(1.0)+mask*coherence*(vec2(density,density*density)-vec2(1.0));
}
