use glam::DVec3;

// Independent one-dimensional Simpson integral of the literal GGX BRDF at
// normal incidence in a uniform radiance field. No importance-sampling mirror.
pub fn uniform_reference(roughness: f64, f0: f64) -> f64 {
    let r = roughness.clamp(0.02, 1.0);
    let a2 = (r * r).max(0.0025).powi(2);
    let k = (r + 1.0).powi(2) / 8.0;
    let f = |mu: f64| {
        let nh = ((1.0 + mu) * 0.5).sqrt();
        let d = a2 / (std::f64::consts::PI * (nh * nh * (a2 - 1.0) + 1.0).powi(2));
        let gl = mu / (mu * (1.0 - k) + k);
        let fresnel = f0 + (1.0 - f0) * (1.0 - nh).powi(5);
        0.5 * std::f64::consts::PI * d * gl * fresnel
    };
    let n = 65536;
    let mut sum = f(0.0) + f(1.0);
    for i in 1..n {
        sum += f(i as f64 / n as f64) * if i % 2 == 0 { 2.0 } else { 4.0 };
    }
    sum / (3.0 * n as f64)
}

pub fn directional_reference(roughness: f64, nv: f64, f0: f64, n: usize) -> f64 {
    if nv < 0.0 {
        return 0.0;
    }
    let eye = DVec3::new((1.0 - nv * nv).sqrt(), 0.0, nv);
    let r = roughness.clamp(0.02, 1.0);
    let a2 = (r * r).max(0.0025).powi(2);
    let k = (r + 1.0).powi(2) / 8.0;
    // Integrate the BRDF after cancelling N.V analytically. This independent
    // solid-angle reference remains defined at the exact grazing limit.
    let gv_over_nv = 1.0 / (nv * (1.0 - k) + k);
    let mut sum = 0.0;
    for y in 0..n {
        let mu = (y as f64 + 0.5) / n as f64;
        let sine = (1.0 - mu * mu).sqrt();
        let gl = mu / (mu * (1.0 - k) + k);
        for x in 0..n {
            let phi = std::f64::consts::TAU * (x as f64 + 0.5) / n as f64;
            let h = (eye + DVec3::new(sine * phi.cos(), sine * phi.sin(), mu)).normalize();
            let d = a2 / (std::f64::consts::PI * (h.z * h.z * (a2 - 1.0) + 1.0).powi(2));
            let fresnel = f0 + (1.0 - f0) * (1.0 - eye.dot(h)).powi(5);
            sum += d * gv_over_nv * gl * fresnel * 0.25;
        }
    }
    sum * std::f64::consts::TAU / (n * n) as f64
}

pub fn average_reference(roughness: f64, f0: f64, n: usize) -> f64 {
    (0..16)
        .map(|i| directional_reference(roughness, ((i as f64 + 0.5) / 16.0).sqrt(), f0, n))
        .sum::<f64>()
        / 16.0
}
