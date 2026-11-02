//! Sample-level visibility gate before an expensive geometry query.
//!
//! A cheaper zero-visibility result cannot be brightened by another blocker.
//! Other estimates need correction only on an illuminated surface and within
//! the explicitly configured normal-exclusion/ray-distance interval. This is
//! not a proof that a lit shadow-map texel or a missing SSR hit is reliable.

/// Returns the ray's minimum SI distance, or `None` to retain the estimate.
/// Inputs belong to the caller's validated light/receiver policy. Invalid
/// estimates/budgets are rejected rather than silently treated as visibility.
pub fn query_min_distance(
    visibility: f64,
    normal_light_cosine: f64,
    normal_exclusion_m: f64,
    max_distance_m: f64,
) -> Result<Option<f64>, &'static str> {
    if !visibility.is_finite()
        || !(0.0..=1.0).contains(&visibility)
        || !normal_light_cosine.is_finite()
        || !normal_exclusion_m.is_finite()
        || normal_exclusion_m < 0.0
        || !max_distance_m.is_finite()
        || max_distance_m <= 0.0
    {
        return Err("invalid visibility estimate or ray interval");
    }
    if visibility == 0.0 || normal_light_cosine <= 0.0 {
        return Ok(None);
    }
    let distance = normal_exclusion_m / normal_light_cosine;
    Ok((distance < max_distance_m).then_some(distance))
}

/// Portable scalar counterpart for visibility consumers. A negative result
/// means keep the cheaper result; no accelerator or backend handle lives here.
pub const QUERY_POLICY_WGSL: &str = r#"
fn fastrt_visibility_min_distance(visibility:f32,cosine:f32,bias:f32,max_distance:f32)->f32 {
    let finite_max=bitcast<f32>(0x7f7fffffu);
    if !(visibility>0. && visibility<=1. && cosine>0. && cosine<=finite_max
        && bias>=0. && bias<=finite_max && max_distance>0. && max_distance<=finite_max) { return -1.; }
    let distance=bias/cosine;
    if !(distance<max_distance) { return -1.; }
    return distance;
}
"#;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cheap_occlusion_backlight_and_excluded_intervals_need_no_ray() {
        assert_eq!(query_min_distance(0., 1., 0.01, 100.), Ok(None));
        assert_eq!(query_min_distance(0.5, -0.5, 0.01, 100.), Ok(None));
        assert_eq!(query_min_distance(1., 0., 0., 100.), Ok(None));
        assert_eq!(query_min_distance(1., 0.01, 1., 100.), Ok(None));
        assert_eq!(query_min_distance(1., 0.5, 0.01, 100.), Ok(Some(0.02)));
        assert_eq!(query_min_distance(0.5, 0.5, 0.01, 100.), Ok(Some(0.02)));
        assert_eq!(query_min_distance(1., 1., 0., 100.), Ok(Some(0.)));
    }

    #[test]
    fn invalid_estimates_do_not_become_a_trusted_miss() {
        for v in [f64::NAN, f64::INFINITY, -1., 2.] {
            assert!(query_min_distance(v, 1., 0.01, 100.).is_err());
        }
        assert!(query_min_distance(1., f64::NAN, 0.01, 100.).is_err());
        assert!(query_min_distance(1., 1., -0.01, 100.).is_err());
        assert!(query_min_distance(1., 1., 0.01, f64::INFINITY).is_err());
        assert!(query_min_distance(1., 1., 0.01, 0.).is_err());
    }
}
