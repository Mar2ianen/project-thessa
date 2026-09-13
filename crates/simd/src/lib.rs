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
        std::arch::is_x86_feature_detected!("avx512f") && std::arch::is_x86_feature_detected!("fma")
    })
}

#[cfg(target_arch = "x86_64")]
fn has_avx2() -> bool {
    use std::sync::OnceLock;
    static HAS: OnceLock<bool> = OnceLock::new();
    *HAS.get_or_init(|| {
        std::arch::is_x86_feature_detected!("avx2") && std::arch::is_x86_feature_detected!("fma")
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
        let bad_mask =
            _mm512_cmp_pd_mask(d2, d2, _CMP_UNORD_Q) | _mm512_cmp_pd_mask(d2, zero, _CMP_EQ_OQ);
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
        let Some(end) = base.checked_add(8) else {
            return false;
        };
        if !(avx512_available()) {
            return false;
        }
        let slices: [&[f64]; 12] = [x0, x1, vx0, vx1, y0, y1, vy0, vy1, z0, z1, vz0, vz1];
        if slices.iter().any(|v| v.len() < end) {
            return false;
        }
        if ox.len() < end || oy.len() < end || oz.len() < end {
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
        let Some(end) = base.checked_add(8) else {
            return false;
        };
        if !(avx512_available()) {
            return false;
        }
        if cx.len() < end
            || cy.len() < end
            || cz.len() < end
            || mu.len() < end
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
        _mm256_storeu_pd(partial.as_mut_ptr(), _mm256_mul_pd(dx, t));
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
        let Some(end) = base.checked_add(4) else {
            return false;
        };
        if !has_avx2() {
            return false;
        }
        let slices: [&[f64]; 12] = [x0, x1, vx0, vx1, y0, y1, vy0, vy1, z0, z1, vz0, vz1];
        if slices.iter().any(|v| v.len() < end) {
            return false;
        }
        if ox.len() < end || oy.len() < end || oz.len() < end {
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
        let Some(end) = base.checked_add(4) else {
            return false;
        };
        if !has_avx2() {
            return false;
        }
        if cx.len() < end
            || cy.len() < end
            || cz.len() < end
            || mu.len() < end
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

#[cfg(test)]
mod boundary_regressions {
    use super::*;

    #[test]
    fn overflowing_or_short_batches_fail_without_touching_output() {
        let input = [1.0; 16];
        for base in [usize::MAX, usize::MAX - 3, usize::MAX - 7, 16] {
            let mut gravity = (7.0, 8.0, 9.0);
            for kernel in [gravity_chunk, gravity_quad] {
                assert!(!kernel(
                    &input,
                    &input,
                    &input,
                    &input,
                    base,
                    0.0,
                    0.0,
                    0.0,
                    &mut gravity
                ));
                assert_eq!(gravity, (7.0, 8.0, 9.0));
            }
            let mut x = [7.0; 16];
            let mut y = [8.0; 16];
            let mut z = [9.0; 16];
            for kernel in [hermite_snapshot_chunk, hermite_snapshot_quad] {
                assert!(!kernel(
                    &input, &input, &input, &input, &input, &input, &input, &input, &input, &input,
                    &input, &input, base, 1.0, 0.5, &mut x, &mut y, &mut z
                ));
                assert_eq!(x, [7.0; 16]);
                assert_eq!(y, [8.0; 16]);
                assert_eq!(z, [9.0; 16]);
            }
        }
    }
}

/// Flat config block for the panel-coefficient kernel: every scalar the
/// analytic model consumes, broadcast-loaded. Control deflection is already
/// folded into `alpha_eff` by the scalar prologue (with separation-dependent
/// effectiveness, so the kernel never branches on control), and lift sign /
/// exposure stay outside in force assembly.
#[derive(Debug, Clone, Copy)]
pub struct AeroKernelParams {
    pub lift_slope: f64,
    pub max_lift: f64,
    pub base_drag: f64,
    pub induced: f64,
    pub wave_coeff: f64,
    pub side_slope: f64,
    pub cm_att: f64,
    pub sep_lift: f64,
    pub sep_drag: f64,
    pub cm_sep: f64,
    pub vortex: f64,
    pub ss_factor: f64,
    pub wave_factor: f64,
    pub beta_floor: f64,
    pub drag_cut: f64,
}

/// 8-wide panel coefficients. Scalar trig prologue (atan2/sincos per lane)
/// stays outside: the kernel consumes alpha, beta, sin/cos and Mach plus
/// per-lane panel scalars, and returns cl/cd/cy/cm. Everything inside is
/// mul/add/min/max/sqrt/div plus Mach-regime masks — no atan2, powf or tanh.
/// Returns `false` when AVX-512 is unavailable (caller falls through to the
/// 4-wide tier, then scalar); per-lane validity is the caller's contract
/// (same checks as the scalar path, run before dispatch).
#[allow(clippy::too_many_arguments)]
pub fn aero_coefficients_chunk(
    alpha_eff: &[f64],
    beta: &[f64],
    sin_e: &[f64],
    cos_e: &[f64],
    sep: &[f64],
    mach: &[f64],
    sweep_cos: &[f64],
    aspect: &[f64],
    interf: &[f64],
    thick: &[f64],
    base: usize,
    params: &AeroKernelParams,
    cl: &mut [f64],
    cd: &mut [f64],
    cy: &mut [f64],
    cm: &mut [f64],
) -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        if !has_avx512() {
            return false;
        }
        let ins: [&[f64]; 10] = [
            alpha_eff, beta, sin_e, cos_e, sep, mach, sweep_cos, aspect, interf, thick,
        ];
        if ins.iter().any(|v| v.len() < base + 8) {
            return false;
        }
        let outs: [&mut [f64]; 4] = [cl, cd, cy, cm];
        if outs.iter().any(|v| v.len() < base + 8) {
            return false;
        }
        unsafe {
            aero8_avx512(
                alpha_eff.as_ptr().add(base),
                beta.as_ptr().add(base),
                sin_e.as_ptr().add(base),
                cos_e.as_ptr().add(base),
                sep.as_ptr().add(base),
                mach.as_ptr().add(base),
                sweep_cos.as_ptr().add(base),
                aspect.as_ptr().add(base),
                interf.as_ptr().add(base),
                thick.as_ptr().add(base),
                params,
                cl.as_mut_ptr().add(base),
                cd.as_mut_ptr().add(base),
                cy.as_mut_ptr().add(base),
                cm.as_mut_ptr().add(base),
            );
        }
        true
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (
            alpha_eff, beta, sin_e, cos_e, sep, mach, sweep_cos, aspect, interf, thick, base,
            params, cl, cd, cy, cm,
        );
        false
    }
}

/// 4-wide variant of [`aero_coefficients_chunk`] for the AVX2 tier.
#[allow(clippy::too_many_arguments)]
pub fn aero_coefficients_quad(
    alpha_eff: &[f64],
    beta: &[f64],
    sin_e: &[f64],
    cos_e: &[f64],
    sep: &[f64],
    mach: &[f64],
    sweep_cos: &[f64],
    aspect: &[f64],
    interf: &[f64],
    thick: &[f64],
    base: usize,
    params: &AeroKernelParams,
    cl: &mut [f64],
    cd: &mut [f64],
    cy: &mut [f64],
    cm: &mut [f64],
) -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        if !has_avx2() {
            return false;
        }
        let ins: [&[f64]; 10] = [
            alpha_eff, beta, sin_e, cos_e, sep, mach, sweep_cos, aspect, interf, thick,
        ];
        if ins.iter().any(|v| v.len() < base + 4) {
            return false;
        }
        let outs: [&mut [f64]; 4] = [cl, cd, cy, cm];
        if outs.iter().any(|v| v.len() < base + 4) {
            return false;
        }
        unsafe {
            aero4_avx2(
                alpha_eff.as_ptr().add(base),
                beta.as_ptr().add(base),
                sin_e.as_ptr().add(base),
                cos_e.as_ptr().add(base),
                sep.as_ptr().add(base),
                mach.as_ptr().add(base),
                sweep_cos.as_ptr().add(base),
                aspect.as_ptr().add(base),
                interf.as_ptr().add(base),
                thick.as_ptr().add(base),
                params,
                cl.as_mut_ptr().add(base),
                cd.as_mut_ptr().add(base),
                cy.as_mut_ptr().add(base),
                cm.as_mut_ptr().add(base),
            );
        }
        true
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (
            alpha_eff, beta, sin_e, cos_e, sep, mach, sweep_cos, aspect, interf, thick, base,
            params, cl, cd, cy, cm,
        );
        false
    }
}

/// 8-wide coefficient core. Mirrors the scalar analytic model branch for
/// branch (tolerance test absorbs FMA contraction); Mach-regime selection
/// is mask blends. All inputs finite per the caller's contract.
#[cfg(target_arch = "x86_64")]
#[allow(clippy::too_many_arguments)]
#[target_feature(enable = "avx512f")]
unsafe fn aero8_avx512(
    alpha_eff: *const f64,
    beta: *const f64,
    sin_e: *const f64,
    cos_e: *const f64,
    sep: *const f64,
    mach: *const f64,
    sweep_cos: *const f64,
    aspect: *const f64,
    interf: *const f64,
    thick: *const f64,
    p: &AeroKernelParams,
    cl_o: *mut f64,
    cd_o: *mut f64,
    cy_o: *mut f64,
    cm_o: *mut f64,
) {
    use std::arch::x86_64::*;
    unsafe {
        // smoothstep(e0,e1,x) = tc^2*(3-2*tc), tc clamped: one FMA
        // for the inner bracket, exact same shape as the scalar path.
        macro_rules! sstep2 {
            ($e0:expr, $e1:expr, $x:expr, $one:expr, $two:expr, $three:expr, $zero:expr) => {{
                let t = _mm512_div_pd(_mm512_sub_pd($x, $e0), _mm512_sub_pd($e1, $e0));
                let tc = _mm512_min_pd(_mm512_max_pd(t, $zero), $one);
                let tc2 = _mm512_mul_pd(tc, tc);
                _mm512_mul_pd(tc2, _mm512_fmsub_pd($three, $one, _mm512_mul_pd($two, tc)))
            }};
        }
        let one = _mm512_set1_pd(1.0);
        let two = _mm512_set1_pd(2.0);
        let three = _mm512_set1_pd(3.0);
        let zero = _mm512_sub_pd(one, one);
        let ae = _mm512_loadu_pd(alpha_eff);
        let sn = _mm512_loadu_pd(sin_e);
        let cs = _mm512_loadu_pd(cos_e);
        let sp = _mm512_loadu_pd(sep);
        let ma = _mm512_loadu_pd(mach);
        let be = _mm512_loadu_pd(beta);
        let sc = _mm512_loadu_pd(sweep_cos);
        let ar = _mm512_loadu_pd(aspect);
        let it = _mm512_loadu_pd(interf);
        let th = _mm512_loadu_pd(thick);

        let gt1 = _mm512_cmp_pd_mask(ma, one, _CMP_GT_OQ);
        let nm = _mm512_mask_blend_pd(gt1, ma, _mm512_mul_pd(ma, sc));
        let sub = _mm512_div_pd(
            _mm512_set1_pd(p.lift_slope),
            _mm512_sqrt_pd(_mm512_max_pd(
                _mm512_sub_pd(one, _mm512_mul_pd(ma, ma)),
                _mm512_set1_pd(p.beta_floor * p.beta_floor),
            )),
        );
        let tra = _mm512_div_pd(_mm512_set1_pd(p.lift_slope), _mm512_set1_pd(p.beta_floor));
        let nm2m1 = _mm512_max_pd(
            _mm512_sub_pd(_mm512_mul_pd(nm, nm), one),
            _mm512_set1_pd(0.05),
        );
        let sup = _mm512_mask_blend_pd(
            gt1,
            tra,
            _mm512_min_pd(
                _mm512_div_pd(_mm512_set1_pd(p.ss_factor), _mm512_sqrt_pd(nm2m1)),
                _mm512_set1_pd(p.lift_slope * 2.5),
            ),
        );
        let m08 = _mm512_set1_pd(0.80);
        let m10 = _mm512_set1_pd(1.00);
        let m12 = _mm512_set1_pd(1.20);
        // Compressible-slope regime selection, narrowest match last: each
        // blend only touches its own band, so subsonic can never be
        // clobbered by the transonic plateau (a previous revision applied
        // the <=1.0 blend after <=0.8 and promoted every subsonic lane).
        let le08 = _mm512_cmp_pd_mask(nm, m08, _CMP_LE_OQ);
        let le10 = _mm512_cmp_pd_mask(nm, m10, _CMP_LE_OQ);
        let ge12 = _mm512_cmp_pd_mask(nm, m12, _CMP_GE_OQ);
        let band_plateau = _kand_mask8(le10, _knot_mask8(le08));
        let band_trans = _kand_mask8(_knot_mask8(le10), _knot_mask8(ge12));
        let mid_t = sstep2!(m10, m12, nm, one, two, three, zero);
        let mid_slope = _mm512_fmadd_pd(_mm512_sub_pd(sup, tra), mid_t, tra);
        let mut slope2d = sub;
        slope2d = _mm512_mask_blend_pd(band_plateau, slope2d, tra);
        slope2d = _mm512_mask_blend_pd(band_trans, slope2d, mid_slope);
        slope2d = _mm512_mask_blend_pd(ge12, slope2d, sup);
        let corr = _mm512_div_pd(
            _mm512_mul_pd(_mm512_set1_pd(std::f64::consts::TAU), ar),
            _mm512_mul_pd(slope2d, sc),
        );
        let two_over = _mm512_div_pd(two, corr);
        let denom = _mm512_fmadd_pd(
            corr,
            _mm512_sqrt_pd(_mm512_fmadd_pd(two_over, two_over, one)),
            two,
        );
        let slope = _mm512_div_pd(
            _mm512_mul_pd(it, _mm512_mul_pd(_mm512_mul_pd(slope2d, corr), sc)),
            denom,
        );
        let linear = _mm512_mul_pd(slope, ae);
        let maxl = _mm512_set1_pd(p.max_lift);
        let x = _mm512_div_pd(linear, maxl);
        let capped = _mm512_mul_pd(
            maxl,
            _mm512_div_pd(x, _mm512_sqrt_pd(_mm512_fmadd_pd(x, x, one))),
        );
        let cl_sep = _mm512_mul_pd(
            _mm512_set1_pd(p.sep_lift),
            _mm512_mul_pd(two, _mm512_mul_pd(sn, cs)),
        );
        let cl = _mm512_fmadd_pd(_mm512_sub_pd(cl_sep, capped), sp, capped);
        let sabs = _mm512_andnot_pd(_mm512_set1_pd(-0.0), sn);
        let clv = _mm512_mul_pd(
            _mm512_set1_pd(p.vortex),
            _mm512_mul_pd(sabs, _mm512_mul_pd(sn, cs)),
        );
        let cl_tot = _mm512_add_pd(cl, clv);
        let cd = _mm512_fmadd_pd(
            _mm512_mul_pd(_mm512_set1_pd(p.induced), _mm512_mul_pd(capped, capped)),
            _mm512_sub_pd(one, sp),
            _mm512_set1_pd(p.base_drag),
        );
        let cd = _mm512_fmadd_pd(
            _mm512_mul_pd(_mm512_set1_pd(p.sep_drag), sp),
            _mm512_mul_pd(sn, sn),
            cd,
        );
        let rise = sstep2!(
            _mm512_set1_pd(0.78),
            _mm512_set1_pd(1.18),
            ma,
            one,
            two,
            three,
            zero
        );
        let ae2 = _mm512_mul_pd(ae, ae);
        let wave = _mm512_mul_pd(
            _mm512_set1_pd(p.wave_coeff),
            _mm512_mul_pd(
                rise,
                _mm512_fmadd_pd(_mm512_set1_pd(1.5), ae2, _mm512_set1_pd(0.12)),
            ),
        );
        let supw_s = sstep2!(m10, m12, nm, one, two, three, zero);
        let nalpha = _mm512_mul_pd(ae, sc);
        let supw = _mm512_mask_blend_pd(
            gt1,
            zero,
            _mm512_div_pd(
                _mm512_mul_pd(
                    _mm512_set1_pd(p.wave_factor * 4.0),
                    _mm512_fmadd_pd(nalpha, nalpha, _mm512_mul_pd(th, th)),
                ),
                _mm512_sqrt_pd(nm2m1),
            ),
        );
        let cd = _mm512_fmadd_pd(supw_s, supw, _mm512_add_pd(cd, wave));
        let cut = _mm512_set1_pd(p.drag_cut);
        let cut_inf = _mm512_cmp_pd_mask(cut, _mm512_set1_pd(f64::INFINITY), _CMP_EQ_OQ);
        let fade_s = sstep2!(cut, _mm512_add_pd(cut, one), ma, one, two, three, zero);
        let fade = _mm512_mask_blend_pd(cut_inf, _mm512_sub_pd(one, fade_s), one);
        let cl_out = _mm512_mul_pd(cl_tot, fade);
        let cy = _mm512_mul_pd(_mm512_set1_pd(p.side_slope), be);
        let cm = _mm512_fmadd_pd(
            _mm512_sub_pd(_mm512_set1_pd(p.cm_sep), _mm512_set1_pd(p.cm_att)),
            sp,
            _mm512_set1_pd(p.cm_att),
        );
        _mm512_storeu_pd(cl_o, cl_out);
        _mm512_storeu_pd(cd_o, cd);
        _mm512_storeu_pd(cy_o, cy);
        _mm512_storeu_pd(cm_o, cm);
    }
}

/// 4-wide coefficient core (AVX2+FMA). Same model as
/// [`aero8_avx512`](aero8_avx512); masks are `blendv` sign-bit selects.
/// Full-precision sqrt+div (AVX2 has no double rsqrt).
#[cfg(target_arch = "x86_64")]
#[allow(clippy::too_many_arguments)]
#[target_feature(enable = "avx2,fma")]
unsafe fn aero4_avx2(
    alpha_eff: *const f64,
    beta: *const f64,
    sin_e: *const f64,
    cos_e: *const f64,
    sep: *const f64,
    mach: *const f64,
    sweep_cos: *const f64,
    aspect: *const f64,
    interf: *const f64,
    thick: *const f64,
    p: &AeroKernelParams,
    cl_o: *mut f64,
    cd_o: *mut f64,
    cy_o: *mut f64,
    cm_o: *mut f64,
) {
    use std::arch::x86_64::*;
    unsafe {
        macro_rules! sstep {
            ($e0:expr, $e1:expr, $x:expr, $one:expr, $two:expr, $three:expr, $zero:expr) => {{
                let t = _mm256_div_pd(_mm256_sub_pd($x, $e0), _mm256_sub_pd($e1, $e0));
                let tc = _mm256_min_pd(_mm256_max_pd(t, $zero), $one);
                let tc2 = _mm256_mul_pd(tc, tc);
                _mm256_mul_pd(tc2, _mm256_fmsub_pd($three, $one, _mm256_mul_pd($two, tc)))
            }};
        }
        let one = _mm256_set1_pd(1.0);
        let two = _mm256_set1_pd(2.0);
        let three = _mm256_set1_pd(3.0);
        let zero = _mm256_sub_pd(one, one);
        let ae = _mm256_loadu_pd(alpha_eff);
        let sn = _mm256_loadu_pd(sin_e);
        let cs = _mm256_loadu_pd(cos_e);
        let sp = _mm256_loadu_pd(sep);
        let ma = _mm256_loadu_pd(mach);
        let be = _mm256_loadu_pd(beta);
        let sc = _mm256_loadu_pd(sweep_cos);
        let ar = _mm256_loadu_pd(aspect);
        let it = _mm256_loadu_pd(interf);
        let th = _mm256_loadu_pd(thick);

        // blendv(cond, false_val, true_val): selects true_val on sign bit.
        macro_rules! sel {
            ($cond:expr, $if_true:expr, $if_false:expr) => {
                _mm256_blendv_pd($if_false, $if_true, $cond)
            };
        }
        let gt1 = _mm256_cmp_pd(ma, one, _CMP_GT_OQ);
        let nm = sel!(gt1, _mm256_mul_pd(ma, sc), ma);
        let sub = _mm256_div_pd(
            _mm256_set1_pd(p.lift_slope),
            _mm256_sqrt_pd(_mm256_max_pd(
                _mm256_sub_pd(one, _mm256_mul_pd(ma, ma)),
                _mm256_set1_pd(p.beta_floor * p.beta_floor),
            )),
        );
        let tra = _mm256_div_pd(_mm256_set1_pd(p.lift_slope), _mm256_set1_pd(p.beta_floor));
        let nm2m1 = _mm256_max_pd(
            _mm256_sub_pd(_mm256_mul_pd(nm, nm), one),
            _mm256_set1_pd(0.05),
        );
        let sup = sel!(
            gt1,
            _mm256_min_pd(
                _mm256_div_pd(_mm256_set1_pd(p.ss_factor), _mm256_sqrt_pd(nm2m1)),
                _mm256_set1_pd(p.lift_slope * 2.5)
            ),
            tra
        );
        let m08 = _mm256_set1_pd(0.80);
        let m10 = _mm256_set1_pd(1.00);
        let m12 = _mm256_set1_pd(1.20);
        // Same band selection as the AVX-512 kernel, built from comparison
        // vectors (blendv keys off the sign bit, so and/andnot compose).
        // A previous revision left the (1.0, 1.2) band at the raw
        // smoothstep factor instead of the blended slope.
        let le08 = _mm256_cmp_pd(nm, m08, _CMP_LE_OQ);
        let le10 = _mm256_cmp_pd(nm, m10, _CMP_LE_OQ);
        let ge12 = _mm256_cmp_pd(nm, m12, _CMP_GE_OQ);
        let not_le08 = _mm256_andnot_pd(le08, _mm256_castsi256_pd(_mm256_set1_epi64x(-1)));
        let not_le10 = _mm256_andnot_pd(le10, _mm256_castsi256_pd(_mm256_set1_epi64x(-1)));
        let not_ge12 = _mm256_andnot_pd(ge12, _mm256_castsi256_pd(_mm256_set1_epi64x(-1)));
        let band_plateau = _mm256_and_pd(le10, not_le08);
        let band_trans = _mm256_and_pd(not_le10, not_ge12);
        let mid_t = sstep!(m10, m12, nm, one, two, three, zero);
        let mid_slope = _mm256_fmadd_pd(_mm256_sub_pd(sup, tra), mid_t, tra);
        let mut slope2d = sub;
        slope2d = sel!(band_plateau, tra, slope2d);
        slope2d = sel!(band_trans, mid_slope, slope2d);
        slope2d = sel!(ge12, sup, slope2d);
        let corr = _mm256_div_pd(
            _mm256_mul_pd(_mm256_set1_pd(std::f64::consts::TAU), ar),
            _mm256_mul_pd(slope2d, sc),
        );
        let two_over = _mm256_div_pd(two, corr);
        let denom = _mm256_fmadd_pd(
            corr,
            _mm256_sqrt_pd(_mm256_fmadd_pd(two_over, two_over, one)),
            two,
        );
        let slope = _mm256_div_pd(
            _mm256_mul_pd(it, _mm256_mul_pd(_mm256_mul_pd(slope2d, corr), sc)),
            denom,
        );
        let linear = _mm256_mul_pd(slope, ae);
        let maxl = _mm256_set1_pd(p.max_lift);
        let x = _mm256_div_pd(linear, maxl);
        let capped = _mm256_mul_pd(
            maxl,
            _mm256_div_pd(x, _mm256_sqrt_pd(_mm256_fmadd_pd(x, x, one))),
        );
        let cl_sep = _mm256_mul_pd(
            _mm256_set1_pd(p.sep_lift),
            _mm256_mul_pd(two, _mm256_mul_pd(sn, cs)),
        );
        let cl = _mm256_fmadd_pd(_mm256_sub_pd(cl_sep, capped), sp, capped);
        let neg_zero = _mm256_set1_pd(-0.0);
        let sabs = _mm256_andnot_pd(neg_zero, sn);
        let clv = _mm256_mul_pd(
            _mm256_set1_pd(p.vortex),
            _mm256_mul_pd(sabs, _mm256_mul_pd(sn, cs)),
        );
        let cl_tot = _mm256_add_pd(cl, clv);
        let cd = _mm256_fmadd_pd(
            _mm256_mul_pd(_mm256_set1_pd(p.induced), _mm256_mul_pd(capped, capped)),
            _mm256_sub_pd(one, sp),
            _mm256_set1_pd(p.base_drag),
        );
        let cd = _mm256_fmadd_pd(
            _mm256_mul_pd(_mm256_set1_pd(p.sep_drag), sp),
            _mm256_mul_pd(sn, sn),
            cd,
        );
        let rise = sstep!(
            _mm256_set1_pd(0.78),
            _mm256_set1_pd(1.18),
            ma,
            one,
            two,
            three,
            zero
        );
        let ae2 = _mm256_mul_pd(ae, ae);
        let wave = _mm256_mul_pd(
            _mm256_set1_pd(p.wave_coeff),
            _mm256_mul_pd(
                rise,
                _mm256_fmadd_pd(_mm256_set1_pd(1.5), ae2, _mm256_set1_pd(0.12)),
            ),
        );
        let supw_s = sstep!(m10, m12, nm, one, two, three, zero);
        let nalpha = _mm256_mul_pd(ae, sc);
        let supw = sel!(
            gt1,
            _mm256_div_pd(
                _mm256_mul_pd(
                    _mm256_set1_pd(p.wave_factor * 4.0),
                    _mm256_fmadd_pd(nalpha, nalpha, _mm256_mul_pd(th, th)),
                ),
                _mm256_sqrt_pd(nm2m1),
            ),
            zero
        );
        let cd = _mm256_fmadd_pd(supw_s, supw, _mm256_add_pd(cd, wave));
        let cut = _mm256_set1_pd(p.drag_cut);
        let cut_inf = _mm256_cmp_pd(cut, _mm256_set1_pd(f64::INFINITY), _CMP_EQ_OQ);
        let fade_s = sstep!(cut, _mm256_add_pd(cut, one), ma, one, two, three, zero);
        let fade = sel!(cut_inf, one, _mm256_sub_pd(one, fade_s));
        let cl_out = _mm256_mul_pd(cl_tot, fade);
        let cy = _mm256_mul_pd(_mm256_set1_pd(p.side_slope), be);
        let cm = _mm256_fmadd_pd(
            _mm256_sub_pd(_mm256_set1_pd(p.cm_sep), _mm256_set1_pd(p.cm_att)),
            sp,
            _mm256_set1_pd(p.cm_att),
        );
        _mm256_storeu_pd(cl_o, cl_out);
        _mm256_storeu_pd(cd_o, cd);
        _mm256_storeu_pd(cy_o, cy);
        _mm256_storeu_pd(cm_o, cm);
    }
}

#[cfg(test)]
mod aero_kernel_tests {
    use super::{AeroKernelParams, aero_coefficients_chunk, aero_coefficients_quad};

    fn default_params() -> AeroKernelParams {
        AeroKernelParams {
            lift_slope: 4.6,
            max_lift: 1.45,
            base_drag: 0.032,
            induced: 0.075,
            wave_coeff: 0.22,
            side_slope: 1.10,
            cm_att: 0.0,
            sep_lift: 1.0,
            sep_drag: 1.9,
            cm_sep: 0.0,
            vortex: 0.4,
            ss_factor: 4.0,
            wave_factor: 1.15,
            beta_floor: 0.6,
            drag_cut: f64::INFINITY,
        }
    }

    /// Scalar oracle of the kernel math (same branch structure, libm trig
    /// stays out — inputs arrive precomputed like in production).
    #[allow(clippy::too_many_arguments)]
    fn scalar_lane(
        p: &AeroKernelParams,
        ae: f64,
        beta: f64,
        sn: f64,
        cs: f64,
        sep: f64,
        ma: f64,
        sweep_cos: f64,
        ar: f64,
        itf: f64,
        th: f64,
    ) -> (f64, f64, f64, f64) {
        fn sstep(e0: f64, e1: f64, x: f64) -> f64 {
            let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
            t * t * (3.0 - 2.0 * t)
        }
        let nm = if ma > 1.0 { ma * sweep_cos } else { ma };
        let sub = p.lift_slope / (1.0 - ma * ma).max(p.beta_floor * p.beta_floor).sqrt();
        let tra = p.lift_slope / p.beta_floor;
        let sup = if nm > 1.0 {
            (p.ss_factor / (nm * nm - 1.0).max(0.05).sqrt()).min(p.lift_slope * 2.5)
        } else {
            tra
        };
        let slope2d = if nm <= 0.80 {
            sub
        } else if nm <= 1.0 {
            tra
        } else if nm >= 1.20 {
            sup
        } else {
            tra + (sup - tra) * sstep(1.0, 1.20, nm)
        };
        let corr = std::f64::consts::TAU * ar / (slope2d * sweep_cos);
        let denom = 2.0 + corr * (1.0 + (2.0 / corr).powi(2)).sqrt();
        let slope = itf * slope2d * corr * sweep_cos / denom;
        let linear = slope * ae;
        let x = linear / p.max_lift;
        let capped = p.max_lift * x / (1.0 + x * x).sqrt();
        let cl_sep = p.sep_lift * 2.0 * sn * cs;
        let cl = capped + (cl_sep - capped) * sep;
        let clv = p.vortex * sn.abs() * sn * cs;
        let cl_tot = cl + clv;
        let cd =
            p.base_drag + p.induced * capped * capped * (1.0 - sep) + p.sep_drag * sep * sn * sn;
        let rise = sstep(0.78, 1.18, ma);
        let nalpha = ae * sweep_cos;
        let supw = if nm > 1.0 {
            p.wave_factor * 4.0 * (nalpha * nalpha + th * th) / (nm * nm - 1.0).max(0.05).sqrt()
        } else {
            0.0
        };
        let wave = p.wave_coeff * (rise * (0.12 + 1.5 * ae * ae)) + sstep(1.0, 1.20, nm) * supw;
        let cd = cd + wave;
        let fade = if p.drag_cut.is_infinite() {
            1.0
        } else {
            1.0 - sstep(p.drag_cut, p.drag_cut + 1.0, ma)
        };
        let cy = p.side_slope * beta;
        let cm = p.cm_att + (p.cm_sep - p.cm_att) * sep;
        (cl_tot * fade, cd, cy, cm)
    }

    #[test]
    fn aero_kernels_match_scalar_oracle() {
        // Sweep attached/stall/separated, sub/trans/super-sonic, alpha past
        // 90 deg, control folded upstream. The 8-wide kernel runs at bases
        // 0 and 8, the 4-wide at 0/4/8/12; `ran_any` records execution so
        // scalar-only machines pass vacuously.
        let p = default_params();
        // (alpha_eff, beta, mach, sweep, aspect)
        let cases: [(f64, f64, f64, f64, f64); 16] = [
            (0.0f64, 0.0, 0.3, 0.0, 2.5),
            (0.15, 0.02, 0.9, 0.1, 4.0),
            (0.5, -0.05, 1.4, -0.2, 1.2),
            (1.2, 0.1, 2.5, 0.3, 8.0),
            (0.05, 0.0, 0.5, 0.0, 2.5),
            (0.9, 0.03, 1.1, 0.05, 3.0),
            (2.0, -0.1, 0.7, 0.4, 1.0),
            (0.4, 0.0, 0.2, -0.1, 6.0),
            (0.1, 0.01, 0.4, 0.0, 2.5),
            (1.1, 0.06, 1.05, 0.15, 5.0),
            (0.7, -0.03, 0.85, 0.0, 2.0),
            (2.6, 0.2, 0.6, -0.3, 1.5),
            (0.15, 0.0, 0.3, 0.1, 4.0),
            (0.55, 0.04, 1.0, 0.2, 2.5),
            (1.5, -0.08, 1.8, 0.0, 3.5),
            (0.3, 0.0, 0.45, 0.05, 7.0),
        ];
        const N: usize = 16;
        let mut alpha = vec![0.0; N];
        let mut beta = vec![0.0; N];
        let mut sin_e = vec![0.0; N];
        let mut cos_e = vec![0.0; N];
        let mut sep = vec![0.0; N];
        let mut mach = vec![0.0; N];
        let mut sweep_cos = vec![0.0; N];
        let mut aspect = vec![0.0; N];
        let mut interf = vec![0.0; N];
        let mut thick = vec![0.0; N];
        for (i, (a, b, m, sw, ar)) in cases.into_iter().enumerate() {
            // Separation from a fixed 0.35 rad stall + 0.1 blend, exactly
            // like the production prologue computes it.
            let t = ((a.abs() - 0.35) / 0.1).clamp(0.0, 1.0);
            sep[i] = t * t * (3.0 - 2.0 * t);
            alpha[i] = a;
            beta[i] = b;
            mach[i] = m;
            sweep_cos[i] = sw.cos();
            aspect[i] = ar;
            interf[i] = 1.0;
            thick[i] = 0.02;
            let (s, c) = a.sin_cos();
            sin_e[i] = s;
            cos_e[i] = c;
        }
        // Each tier writes its own buffers: an earlier revision ran the
        // 4-wide kernel over the 8-wide results, so the 8-wide tier was
        // never actually compared (this hid a subsonic-slope bug there).
        let mut cl8 = vec![f64::NAN; N];
        let mut cd8 = vec![f64::NAN; N];
        let mut cy8 = vec![f64::NAN; N];
        let mut cm8 = vec![f64::NAN; N];
        let mut cl4 = vec![f64::NAN; N];
        let mut cd4 = vec![f64::NAN; N];
        let mut cy4 = vec![f64::NAN; N];
        let mut cm4 = vec![f64::NAN; N];
        let mut ran8 = false;
        let mut ran4 = false;
        for base in [0usize, 8] {
            if aero_coefficients_chunk(
                &alpha, &beta, &sin_e, &cos_e, &sep, &mach, &sweep_cos, &aspect, &interf, &thick,
                base, &p, &mut cl8, &mut cd8, &mut cy8, &mut cm8,
            ) {
                ran8 = true;
            }
        }
        for base in [0usize, 4, 8, 12] {
            if aero_coefficients_quad(
                &alpha, &beta, &sin_e, &cos_e, &sep, &mach, &sweep_cos, &aspect, &interf, &thick,
                base, &p, &mut cl4, &mut cd4, &mut cy4, &mut cm4,
            ) {
                ran4 = true;
            }
        }
        // Full-model cross-check per lane against the scalar oracle.
        for i in 0..N {
            let (e_cl, e_cd, e_cy, e_cm) = scalar_lane(
                &p,
                alpha[i],
                beta[i],
                sin_e[i],
                cos_e[i],
                sep[i],
                mach[i],
                sweep_cos[i],
                aspect[i],
                interf[i],
                thick[i],
            );
            for (tag, ran, g_cl, g_cd, g_cy, g_cm) in [
                ("avx512", ran8, cl8[i], cd8[i], cy8[i], cm8[i]),
                ("avx2", ran4, cl4[i], cd4[i], cy4[i], cm4[i]),
            ] {
                if !ran {
                    continue;
                }
                let scale = (e_cl.abs() + e_cd.abs() + e_cy.abs() + e_cm.abs() + 1e-9).max(1e-9);
                let err = (g_cl - e_cl).abs()
                    + (g_cd - e_cd).abs()
                    + (g_cy - e_cy).abs()
                    + (g_cm - e_cm).abs();
                assert!(
                    err / scale < 1e-9,
                    "{tag} lane {i} diverged: got ({g_cl},{g_cd},{g_cy},{g_cm}) want ({e_cl},{e_cd},{e_cy},{e_cm})"
                );
            }
        }
        eprintln!("aero kernel self-test ran8={ran8} ran4={ran4}");
        assert!(ran8 || ran4, "no SIMD tier executed on this machine");
    }
}
