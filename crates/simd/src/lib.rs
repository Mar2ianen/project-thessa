//! Optional AVX-512 math kernels for Project Thessa numerical loops.
//!
//! Two per-step costs dominate a long ephemeris bake: Hermite interpolation
//! of body centers and the gravity accumulation (one sqrt+recip per body).
//! Both are data-parallel across bodies, so 8-wide f64 kernels with runtime
//! dispatch buy a real multiple while the scalar path stays the portable
//! baseline and cross-check oracle.
//!
//! `unsafe` is isolated in this MIT math crate (raw SIMD loads/stores with
//! bounds checked once at each safe boundary); `thessa-sim-core` keeps its
//! `#![forbid(unsafe_code)]`.
//!
//! Precision: Hermite matches the scalar op order structurally (tolerance
//! ~1e-15 relative); gravity refines `rsqrt14` twice with Newton iterations
//! to full-double accuracy. Dispatch is deterministic per machine;
//! cross-machine bits may differ.

/// Runtime AVX-512 gate. `is_x86_feature_detected` executes cpuid, so call
/// sites cache this once per bake, never per step.
#[cfg(target_arch = "x86_64")]
pub fn avx512_available() -> bool {
    std::arch::is_x86_feature_detected!("avx512f")
        && std::arch::is_x86_feature_detected!("fma")
}

/// Non-x86 builds compile to the scalar path only.
#[cfg(not(target_arch = "x86_64"))]
pub fn avx512_available() -> bool {
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
