//! Optional SIMD math kernels for Project Thessa numerical loops (AVX-512
//! 8-wide with an AVX2 4-wide fallback — AVX2 ships in nearly every x86-64
//! CPU built in the last decade).
//!
//! Two per-step costs dominate a long ephemeris bake: Hermite interpolation
//! of body centers and the gravity accumulation (one sqrt+recip per body).
//! Both are data-parallel across bodies, so wide f64 kernels with runtime
//! dispatch buy a real multiple while the scalar path stays the portable
//! baseline and cross-check oracle.
//!
//! `unsafe` is isolated in this MIT math crate (raw SIMD loads/stores with
//! bounds checked once at each safe boundary); `thessa-sim-core` keeps its
//! `#![forbid(unsafe_code)]`.
//!
//! Precision: Hermite matches the scalar op order structurally (tolerance
//! ~1e-15 relative); the 512-bit gravity kernel refines `rsqrt14` twice
//! with Newton iterations, the 256-bit one uses full-precision sqrt+div.
//! Dispatch is deterministic per machine; cross-machine bits may differ.

/// Independent tier flags, resolved once per process: `is_x86_feature_detected`
/// executes cpuid, so never probe per chunk. An AVX-512 machine also reports
/// AVX2 (both kernels stay exercisable for tests and partial tails).
#[cfg(target_arch = "x86_64")]
fn has_avx512() -> bool {
    use std::sync::OnceLock;
    static HAS: OnceLock<bool> = OnceLock::new();
    *HAS.get_or_init(|| {
        std::arch::is_x86_feature_detected!("avx512f")
            && std::arch::is_x86_feature_detected!("fma")
    })
}

#[cfg(target_arch = "x86_64")]
fn has_avx2() -> bool {
    use std::sync::OnceLock;
    static HAS: OnceLock<bool> = OnceLock::new();
    *HAS.get_or_init(|| {
        std::arch::is_x86_feature_detected!("avx2")
            && std::arch::is_x86_feature_detected!("fma")
    })
}

/// Runtime AVX-512 gate (kept for callers/tests probing the top tier).
#[cfg(target_arch = "x86_64")]
pub fn avx512_available() -> bool {
    has_avx512()
}

/// Runtime AVX2 gate: true on nearly every x86-64 CPU of the last decade
/// (and on every AVX-512 machine).
#[cfg(target_arch = "x86_64")]
pub fn avx2_available() -> bool {
    has_avx2()
}

/// Non-x86 builds compile to the scalar path only.
#[cfg(not(target_arch = "x86_64"))]
pub fn avx512_available() -> bool {
    false
}

/// Non-x86 builds compile to the scalar path only.
#[cfg(not(target_arch = "x86_64"))]
pub fn avx2_available() -> bool {
    false
}

/// 8-wide Hermite positions for one node interval, component-major inputs.
///
/// `x0`/`x1`/`vx0`/`vx1` (etc.) are runs of one node's bodies at `base`;
/// `out` receives centers. Caller guarantees `base + 8` in bounds on every
/// slice. Returns the same values as the scalar loop up to FMA contraction.
#[cfg(target_arch = "x86_64")]
#[allow(clippy::too_many_arguments)]
#[target_feature(enable = "avx512f")]
unsafe fn hermite8_avx512(
    x0: *const f64,
    x1: *const f64,
    vx0: *const f64,
    vx1: *const f64,
    y0: *const f64,
    y1: *const f64,
    vy0: *const f64,
    vy1: *const f64,
    z0: *const f64,
    z1: *const f64,
    vz0: *const f64,
    vz1: *const f64,
    h: f64,
    s: f64,
    ox: *mut f64,
    oy: *mut f64,
    oz: *mut f64,
) {
    use std::arch::x86_64::*;
    unsafe {
    let hn = _mm512_set1_pd(h);
    let sn = _mm512_set1_pd(s);
    let two = _mm512_set1_pd(2.0);
    let three = _mm512_set1_pd(3.0);
    // One component: a = 3*dx - h*(2*v0+v1); b = h*(v0+v1) - 2*dx;
    // p = c0 + s*(h*v0 + s*(a + s*b)).
    macro_rules! component {
        ($c0:expr, $c1:expr, $vc0:expr, $vc1:expr, $o:expr) => {{
            let c0 = _mm512_loadu_pd($c0);
            let c1 = _mm512_loadu_pd($c1);
            let v0 = _mm512_loadu_pd($vc0);
            let v1 = _mm512_loadu_pd($vc1);
            let dx = _mm512_sub_pd(c1, c0);
            let t = _mm512_fmadd_pd(two, v0, v1);
            let a = _mm512_fmsub_pd(three, dx, _mm512_mul_pd(hn, t));
            let u = _mm512_add_pd(v0, v1);
            let b = _mm512_fmsub_pd(hn, u, _mm512_mul_pd(two, dx));
            let inner = _mm512_fmadd_pd(sn, b, a);
            let mid = _mm512_fmadd_pd(sn, inner, _mm512_mul_pd(hn, v0));
            _mm512_storeu_pd($o, _mm512_fmadd_pd(sn, mid, c0));
        }};
    }
    component!(x0, x1, vx0, vx1, ox);
    component!(y0, y1, vy0, vy1, oy);
    component!(z0, z1, vz0, vz1, oz);
    }
}

/// Gravity terms for 8 bodies: `term = (c - p) * (mu * inv(d^2)^3)`.
/// `rsqrt14` plus two Newton refinements reaches full-double accuracy.
/// Returns `None` when any lane is singular/non-finite (caller falls back
/// to scalar, preserving exact `None` semantics); otherwise the three
/// partial sums for the caller to reduce in order.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx512f")]
unsafe fn gravity8_avx512(
    cx: *const f64,
    cy: *const f64,
    cz: *const f64,
    mu: *const f64,
    px: f64,
    py: f64,
    pz: f64,
) -> Option<(f64, f64, f64)> {
    use std::arch::x86_64::*;
    unsafe {
    let vx = _mm512_loadu_pd(cx);
    let vy = _mm512_loadu_pd(cy);
    let vz = _mm512_loadu_pd(cz);
    let m = _mm512_loadu_pd(mu);
    let dx = _mm512_sub_pd(vx, _mm512_set1_pd(px));
    let dy = _mm512_sub_pd(vy, _mm512_set1_pd(py));
    let dz = _mm512_sub_pd(vz, _mm512_set1_pd(pz));
    let d2 = _mm512_fmadd_pd(dx, dx, _mm512_fmadd_pd(dy, dy, _mm512_mul_pd(dz, dz)));
    // Singular (d2 == 0) or non-finite lanes invalidate the whole eval,
    // exactly like the scalar early-`None`.
    let zero = _mm512_setzero_pd();
    let bad_mask = _mm512_cmp_pd_mask(d2, d2, _CMP_UNORD_Q)
        | _mm512_cmp_pd_mask(d2, zero, _CMP_EQ_OQ);
    if bad_mask != 0 {
        return None;
    }
    let mut x = _mm512_rsqrt14_pd(d2);
    let half = _mm512_set1_pd(0.5);
    let one_half = _mm512_set1_pd(1.5);
    for _ in 0..2 {
        let x2 = _mm512_mul_pd(x, x);
        // x *= 1.5 - 0.5*d2*x2
        x = _mm512_mul_pd(x, _mm512_fnmadd_pd(half, _mm512_mul_pd(d2, x2), one_half));
    }
    if _mm512_cmp_pd_mask(x, x, _CMP_UNORD_Q) != 0 {
        return None;
    }
    let inv2 = _mm512_mul_pd(x, x);
    let inv3 = _mm512_mul_pd(inv2, x);
    let t = _mm512_mul_pd(m, inv3);
    let sx = _mm512_reduce_add_pd(_mm512_mul_pd(dx, t));
    let sy = _mm512_reduce_add_pd(_mm512_mul_pd(dy, t));
    let sz = _mm512_reduce_add_pd(_mm512_mul_pd(dz, t));
    if !(sx.is_finite() && sy.is_finite() && sz.is_finite()) {
        return None;
    }
    Some((sx, sy, sz))
    }
}

/// Safe wrappers with bounds checking; scalar fallback lives in `table`.
/// Each returns `false` when the caller must take the scalar path instead
/// (unsupported arch at runtime, or a singular lane).
/// Raw-slice batch entry: 12 component runs in, 3 out. Only 8-wide
/// full lanes run the kernel; the tail stays scalar in the caller.
#[allow(clippy::too_many_arguments)]
pub fn hermite_snapshot_chunk(
    x0: &[f64],
    x1: &[f64],
    vx0: &[f64],
    vx1: &[f64],
    y0: &[f64],
    y1: &[f64],
    vy0: &[f64],
    vy1: &[f64],
    z0: &[f64],
    z1: &[f64],
    vz0: &[f64],
    vz1: &[f64],
    base: usize,
    h: f64,
    s: f64,
    ox: &mut [f64],
    oy: &mut [f64],
    oz: &mut [f64],
) -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        if !(avx512_available()) {
            return false;
        }
        let slices: [&[f64]; 12] = [x0, x1, vx0, vx1, y0, y1, vy0, vy1, z0, z1, vz0, vz1];
        if slices.iter().any(|v| v.len() < base + 8) {
            return false;
        }
        if ox.len() < base + 8 || oy.len() < base + 8 || oz.len() < base + 8 {
            return false;
        }
        if !(h.is_finite() && s.is_finite()) {
            return false;
        }
        unsafe {
            hermite8_avx512(
                x0.as_ptr().add(base),
                x1.as_ptr().add(base),
                vx0.as_ptr().add(base),
                vx1.as_ptr().add(base),
                y0.as_ptr().add(base),
                y1.as_ptr().add(base),
                vy0.as_ptr().add(base),
                vy1.as_ptr().add(base),
                z0.as_ptr().add(base),
                z1.as_ptr().add(base),
                vz0.as_ptr().add(base),
                vz1.as_ptr().add(base),
                h,
                s,
                ox.as_mut_ptr().add(base),
                oy.as_mut_ptr().add(base),
                oz.as_mut_ptr().add(base),
            );
        }
        true
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (
            x0, x1, vx0, vx1, y0, y1, vy0, vy1, z0, z1, vz0, vz1, base, h, s, ox, oy, oz,
        );
        false
    }
}

#[allow(clippy::too_many_arguments)]
pub fn gravity_chunk(
    cx: &[f64],
    cy: &[f64],
    cz: &[f64],
    mu: &[f64],
    base: usize,
    px: f64,
    py: f64,
    pz: f64,
    out: &mut (f64, f64, f64),
) -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        if !(avx512_available()) {
            return false;
        }
        if cx.len() < base + 8
            || cy.len() < base + 8
            || cz.len() < base + 8
            || mu.len() < base + 8
            || !(px.is_finite() && py.is_finite() && pz.is_finite())
        {
            return false;
        }
        let result = unsafe {
            gravity8_avx512(
                cx.as_ptr().add(base),
                cy.as_ptr().add(base),
                cz.as_ptr().add(base),
                mu.as_ptr().add(base),
                px,
                py,
                pz,
            )
        };
        match result {
            Some((sx, sy, sz)) => {
                out.0 += sx;
                out.1 += sy;
                out.2 += sz;
                true
            }
            None => false,
        }
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (cx, cy, cz, mu, base, px, py, pz, out);
        false
    }
}

/// 4-wide Hermite positions (AVX2+FMA fallback for the 8-wide kernel).
/// Same contract as the 8-wide entry: `base + 4` in bounds on every slice.
#[cfg(target_arch = "x86_64")]
#[allow(clippy::too_many_arguments)]
#[target_feature(enable = "avx2,fma")]
unsafe fn hermite4_avx2(
    x0: *const f64,
    x1: *const f64,
    vx0: *const f64,
    vx1: *const f64,
    y0: *const f64,
    y1: *const f64,
    vy0: *const f64,
    vy1: *const f64,
    z0: *const f64,
    z1: *const f64,
    vz0: *const f64,
    vz1: *const f64,
    h: f64,
    s: f64,
    ox: *mut f64,
    oy: *mut f64,
    oz: *mut f64,
) {
    use std::arch::x86_64::*;
    unsafe {
        let hn = _mm256_set1_pd(h);
        let sn = _mm256_set1_pd(s);
        let two = _mm256_set1_pd(2.0);
        let three = _mm256_set1_pd(3.0);
        macro_rules! component {
            ($c0:expr, $c1:expr, $vc0:expr, $vc1:expr, $o:expr) => {{
                let c0 = _mm256_loadu_pd($c0);
                let c1 = _mm256_loadu_pd($c1);
                let v0 = _mm256_loadu_pd($vc0);
                let v1 = _mm256_loadu_pd($vc1);
                let dx = _mm256_sub_pd(c1, c0);
                let t = _mm256_fmadd_pd(two, v0, v1);
                let a = _mm256_fmsub_pd(three, dx, _mm256_mul_pd(hn, t));
                let u = _mm256_add_pd(v0, v1);
                let b = _mm256_fmsub_pd(hn, u, _mm256_mul_pd(two, dx));
                let inner = _mm256_fmadd_pd(sn, b, a);
                let mid = _mm256_fmadd_pd(sn, inner, _mm256_mul_pd(hn, v0));
                _mm256_storeu_pd($o, _mm256_fmadd_pd(sn, mid, c0));
            }};
        }
        component!(x0, x1, vx0, vx1, ox);
        component!(y0, y1, vy0, vy1, oy);
        component!(z0, z1, vz0, vz1, oz);
    }
}

/// 4-wide gravity terms (AVX2 fallback). Full-precision sqrt+div (AVX2 has
/// no double rsqrt); `None` on any singular/non-finite lane, like the
/// 8-wide kernel.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2,fma")]
unsafe fn gravity4_avx2(
    cx: *const f64,
    cy: *const f64,
    cz: *const f64,
    mu: *const f64,
    px: f64,
    py: f64,
    pz: f64,
) -> Option<(f64, f64, f64)> {
    use std::arch::x86_64::*;
    unsafe {
        let vx = _mm256_loadu_pd(cx);
        let vy = _mm256_loadu_pd(cy);
        let vz = _mm256_loadu_pd(cz);
        let m = _mm256_loadu_pd(mu);
        let dx = _mm256_sub_pd(vx, _mm256_set1_pd(px));
        let dy = _mm256_sub_pd(vy, _mm256_set1_pd(py));
        let dz = _mm256_sub_pd(vz, _mm256_set1_pd(pz));
        let d2 = _mm256_fmadd_pd(dx, dx, _mm256_fmadd_pd(dy, dy, _mm256_mul_pd(dz, dz)));
        // NaN lanes or exact-zero distance invalidate the eval.
        if _mm256_movemask_pd(_mm256_cmp_pd(d2, d2, _CMP_UNORD_Q)) != 0 {
            return None;
        }
        if _mm256_movemask_pd(_mm256_cmp_pd(d2, _mm256_setzero_pd(), _CMP_EQ_OQ)) != 0 {
            return None;
        }
        let inv = _mm256_div_pd(_mm256_set1_pd(1.0), _mm256_sqrt_pd(d2));
        let inv2 = _mm256_mul_pd(inv, inv);
        let inv3 = _mm256_mul_pd(inv2, inv);
        let t = _mm256_mul_pd(m, inv3);
        let mut partial = [0.0f64; 4];
        _mm256_storeu_pd(
            partial.as_mut_ptr(),
            _mm256_mul_pd(dx, t),
        );
        let sx: f64 = partial.iter().sum();
        _mm256_storeu_pd(partial.as_mut_ptr(), _mm256_mul_pd(dy, t));
        let sy: f64 = partial.iter().sum();
        _mm256_storeu_pd(partial.as_mut_ptr(), _mm256_mul_pd(dz, t));
        let sz: f64 = partial.iter().sum();
        if !(sx.is_finite() && sy.is_finite() && sz.is_finite()) {
            return None;
        }
        Some((sx, sy, sz))
    }
}

/// 4-wide Hermite batch entry: same contract as the 8-wide one, `base + 4`
/// in bounds. Returns `false` when the caller must take the scalar path.
#[allow(clippy::too_many_arguments)]
pub fn hermite_snapshot_quad(
    x0: &[f64],
    x1: &[f64],
    vx0: &[f64],
    vx1: &[f64],
    y0: &[f64],
    y1: &[f64],
    vy0: &[f64],
    vy1: &[f64],
    z0: &[f64],
    z1: &[f64],
    vz0: &[f64],
    vz1: &[f64],
    base: usize,
    h: f64,
    s: f64,
    ox: &mut [f64],
    oy: &mut [f64],
    oz: &mut [f64],
) -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        if !has_avx2() {
            return false;
        }
        let slices: [&[f64]; 12] = [x0, x1, vx0, vx1, y0, y1, vy0, vy1, z0, z1, vz0, vz1];
        if slices.iter().any(|v| v.len() < base + 4) {
            return false;
        }
        if ox.len() < base + 4 || oy.len() < base + 4 || oz.len() < base + 4 {
            return false;
        }
        if !(h.is_finite() && s.is_finite()) {
            return false;
        }
        unsafe {
            hermite4_avx2(
                x0.as_ptr().add(base),
                x1.as_ptr().add(base),
                vx0.as_ptr().add(base),
                vx1.as_ptr().add(base),
                y0.as_ptr().add(base),
                y1.as_ptr().add(base),
                vy0.as_ptr().add(base),
                vy1.as_ptr().add(base),
                z0.as_ptr().add(base),
                z1.as_ptr().add(base),
                vz0.as_ptr().add(base),
                vz1.as_ptr().add(base),
                h,
                s,
                ox.as_mut_ptr().add(base),
                oy.as_mut_ptr().add(base),
                oz.as_mut_ptr().add(base),
            );
        }
        true
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (
            x0, x1, vx0, vx1, y0, y1, vy0, vy1, z0, z1, vz0, vz1, base, h, s, ox, oy, oz,
        );
        false
    }
}

/// 4-wide gravity batch entry: same contract as the 8-wide one.
#[allow(clippy::too_many_arguments)]
pub fn gravity_quad(
    cx: &[f64],
    cy: &[f64],
    cz: &[f64],
    mu: &[f64],
    base: usize,
    px: f64,
    py: f64,
    pz: f64,
    out: &mut (f64, f64, f64),
) -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        if !has_avx2() {
            return false;
        }
        if cx.len() < base + 4
            || cy.len() < base + 4
            || cz.len() < base + 4
            || mu.len() < base + 4
            || !(px.is_finite() && py.is_finite() && pz.is_finite())
        {
            return false;
        }
        let result = unsafe {
            gravity4_avx2(
                cx.as_ptr().add(base),
                cy.as_ptr().add(base),
                cz.as_ptr().add(base),
                mu.as_ptr().add(base),
                px,
                py,
                pz,
            )
        };
        match result {
            Some((sx, sy, sz)) => {
                out.0 += sx;
                out.1 += sy;
                out.2 += sz;
                true
            }
            None => false,
        }
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (cx, cy, cz, mu, base, px, py, pz, out);
        false
    }
}

#[cfg(test)]
mod tests {
    /// Deterministic pseudo-random f64 in [-scale, scale]: LCG, no deps.
    fn mkvec(rng: &mut u64, scale: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|_| {
                *rng = rng
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                (((*rng >> 11) as f64) / ((1u64 << 53) as f64) - 0.5) * 2.0 * scale
            })
            .collect()
    }

    fn scalar_hermite(x0: f64, x1: f64, vx0: f64, vx1: f64, h: f64, s: f64) -> f64 {
        let dx = x1 - x0;
        let t = 2.0 * vx0 + vx1;
        let a = 3.0 * dx - h * t;
        let u = vx0 + vx1;
        let b = h * u - 2.0 * dx;
        let inner = a + s * b;
        let mid = h * vx0 + s * inner;
        x0 + s * mid
    }

    fn scalar_gravity(
        cx: &[f64],
        cy: &[f64],
        cz: &[f64],
        mu: &[f64],
        px: f64,
        py: f64,
        pz: f64,
    ) -> (f64, f64, f64) {
        let mut total = (0.0, 0.0, 0.0);
        for i in 0..cx.len() {
            let dx = cx[i] - px;
            let dy = cy[i] - py;
            let dz = cz[i] - pz;
            let d2 = dx * dx + dy * dy + dz * dz;
            let inv = 1.0 / d2.sqrt();
            let t = mu[i] * inv * inv * inv;
            total.0 += dx * t;
            total.1 += dy * t;
            total.2 += dz * t;
        }
        total
    }

    #[test]
    fn kernels_match_scalar_reference() {
        // Every available tier (8-wide, 4-wide) against the scalar oracle.
        // On machines without AVX the wrappers return false and the asserts
        // below are skipped; `ran_any` records what actually executed.
        let mut rng = 0x12345678u64;
        const N: usize = 16;
        let x0 = mkvec(&mut rng, 1.0e9, N);
        let x1 = mkvec(&mut rng, 1.0e9, N);
        let vx0 = mkvec(&mut rng, 1.0e4, N);
        let vx1 = mkvec(&mut rng, 1.0e4, N);
        let y0 = mkvec(&mut rng, 1.0e9, N);
        let y1 = mkvec(&mut rng, 1.0e9, N);
        let vy0 = mkvec(&mut rng, 1.0e4, N);
        let vy1 = mkvec(&mut rng, 1.0e4, N);
        let z0 = mkvec(&mut rng, 1.0e9, N);
        let z1 = mkvec(&mut rng, 1.0e9, N);
        let vz0 = mkvec(&mut rng, 1.0e4, N);
        let vz1 = mkvec(&mut rng, 1.0e4, N);
        let (h, s) = (40.0, 0.37);
        let mut ox = vec![0.0; N];
        let mut oy = vec![0.0; N];
        let mut oz = vec![0.0; N];
        let check_hermite = |ox: &[f64], xs: &[(&Vec<f64>, &Vec<f64>, &Vec<f64>, &Vec<f64>)], base: usize, width: usize, tag: &str| {
            for i in base..base + width {
                let (a0, a1, b0, b1) = (&xs[0].0[i], &xs[0].1[i], &xs[0].2[i], &xs[0].3[i]);
                let expected = scalar_hermite(*a0, *a1, *b0, *b1, h, s);
                let scale = expected.abs().max(1.0);
                assert!(
                    (ox[i] - expected).abs() / scale < 1e-12,
                    "{tag} lane {i} diverged"
                );
            }
        };
        let _ = &check_hermite;
        let mut ran_any = false;
        // 8-wide Hermite at bases 0 and 8.
        for base in [0usize, 8] {
            if super::hermite_snapshot_chunk(
                &x0, &x1, &vx0, &vx1, &y0, &y1, &vy0, &vy1, &z0, &z1, &vz0, &vz1, base, h, s,
                &mut ox, &mut oy, &mut oz,
            ) {
                ran_any = true;
                for (o, a0, a1, b0, b1, tag) in [
                    (&ox, &x0, &x1, &vx0, &vx1, "hermite8.x"),
                    (&oy, &y0, &y1, &vy0, &vy1, "hermite8.y"),
                    (&oz, &z0, &z1, &vz0, &vz1, "hermite8.z"),
                ] {
                    for i in base..base + 8 {
                        let expected = scalar_hermite(a0[i], a1[i], b0[i], b1[i], h, s);
                        let scale = expected.abs().max(1.0);
                        assert!(
                            (o[i] - expected).abs() / scale < 1e-12,
                            "{tag} lane {i} diverged"
                        );
                    }
                }
            }
        }
        // 4-wide Hermite at bases 0, 4, 8, 12.
        for base in [0usize, 4, 8, 12] {
            if super::hermite_snapshot_quad(
                &x0, &x1, &vx0, &vx1, &y0, &y1, &vy0, &vy1, &z0, &z1, &vz0, &vz1, base, h, s,
                &mut ox, &mut oy, &mut oz,
            ) {
                ran_any = true;
                for i in base..base + 4 {
                    let expected = scalar_hermite(x0[i], x1[i], vx0[i], vx1[i], h, s);
                    let scale = expected.abs().max(1.0);
                    assert!(
                        (ox[i] - expected).abs() / scale < 1e-12,
                        "hermite4 lane {i} diverged"
                    );
                }
            }
        }
        // Gravity, both widths, compared over their exact lane ranges.
        let mu = mkvec(&mut rng, 1.0e13, N);
        let (px, py, pz) = (1.0e8, -2.0e8, 3.0e8);
        for (base, width, tag) in [(0usize, 8usize, "gravity8"), (8, 8, "gravity8b")] {
            let mut out = (0.0, 0.0, 0.0);
            if super::gravity_chunk(&x0, &y0, &z0, &mu, base, px, py, pz, &mut out) {
                ran_any = true;
                let e = scalar_gravity(
                    &x0[base..base + width],
                    &y0[base..base + width],
                    &z0[base..base + width],
                    &mu[base..base + width],
                    px,
                    py,
                    pz,
                );
                let scale = (e.0.abs() + e.1.abs() + e.2.abs()).max(1e-30);
                let err = (out.0 - e.0).abs() + (out.1 - e.1).abs() + (out.2 - e.2).abs();
                assert!(err / scale < 1e-12, "{tag} diverged: {err:e}");
            }
        }
        for base in [0usize, 4, 8, 12] {
            let mut out = (0.0, 0.0, 0.0);
            if super::gravity_quad(&x0, &y0, &z0, &mu, base, px, py, pz, &mut out) {
                ran_any = true;
                let e = scalar_gravity(
                    &x0[base..base + 4],
                    &y0[base..base + 4],
                    &z0[base..base + 4],
                    &mu[base..base + 4],
                    px,
                    py,
                    pz,
                );
                let scale = (e.0.abs() + e.1.abs() + e.2.abs()).max(1e-30);
                let err = (out.0 - e.0).abs() + (out.1 - e.1).abs() + (out.2 - e.2).abs();
                assert!(err / scale < 1e-12, "gravity4 diverged: {err:e}");
            }
        }
        eprintln!("simd kernel self-test ran_any={ran_any}");
        assert!(ran_any, "no SIMD tier executed on this machine");
    }
}
