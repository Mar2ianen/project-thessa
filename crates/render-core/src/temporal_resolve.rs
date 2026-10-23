//! Backend-neutral temporal-resolve policy.
//!
//! This is a CPU reference for the future GPU pass, not a renderer. It defines
//! how a reprojected history sample is validated, bounded and blended. Texture
//! allocation, sampling, previous-depth storage and presentation remain the
//! renderer's responsibility.
//!
//! Two history-grid conventions are supported:
//!
//! - [`HistoryGrid::ResolvedUnjittered`] is the preferred native TAA path. The
//!   previous resolve was accumulated at pixel centres, so colour is sampled in
//!   unjittered UV and the previous jitter is needed only to address the
//!   previous depth buffer.
//! - [`HistoryGrid::JitteredScene`] samples a raw previous raster image. Colour
//!   and depth are both addressed with the previous jitter applied.
//!
//! Motion is always `previous_unjittered_uv - current_unjittered_uv`. A caller
//! running at a current raster pixel must remove current jitter before using
//! this contract; [`history_uv`] does that once for both conventions.

use crate::temporal::JitterOffset;

/// Maximum accepted history age. Older history is still valid, but its weight
/// stops changing so a long stationary view cannot acquire unbounded inertia.
pub const MAX_HISTORY_AGE: u32 = 64;

/// Default neighbourhood standard-deviation multiplier for colour clipping.
pub const DEFAULT_COLOR_SIGMA: f32 = 1.0;

/// Minimum accepted history weight. This is a responsiveness policy, not a
/// physical quantity: even a long valid history must yield to a changing image.
pub const DEFAULT_MIN_HISTORY_WEIGHT: f32 = 0.05;

/// Coordinate convention of the previous colour history texture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoryGrid {
    /// Previous resolved image at unjittered pixel centres.
    ResolvedUnjittered,
    /// Raw previous raster image produced through its jittered projection.
    JitteredScene,
}

/// Resolved history addresses for one current raster pixel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HistoryUv {
    /// Unjittered previous-frame UV of the same surface point.
    pub unjittered: [f64; 2],
    /// Colour lookup UV for the selected [`HistoryGrid`].
    pub color: [f64; 2],
    /// Previous depth lookup UV. Previous depth is always raster data, so this
    /// always includes the previous jitter.
    pub depth: [f64; 2],
}

/// Compute history lookup UVs for a current raster pixel.
///
/// `current_raster_uv` is the pixel centre in the currently jittered raster.
/// `motion_uv` is the jitter-free velocity from the current unjittered UV to the
/// previous unjittered UV. Returns `None` for non-finite input, an invalid
/// extent, or a jitter outside [`crate::temporal::MAX_JITTER_PX`]. The result may
/// lie outside the image; use [`sampleable`] before sampling.
pub fn history_uv(
    current_raster_uv: [f64; 2],
    motion_uv: [f64; 2],
    current_jitter_px: JitterOffset,
    previous_jitter_px: JitterOffset,
    extent: [u32; 2],
    grid: HistoryGrid,
) -> Option<HistoryUv> {
    if !current_raster_uv
        .iter()
        .chain(motion_uv.iter())
        .all(|value| value.is_finite())
        || extent[0] == 0
        || extent[1] == 0
    {
        return None;
    }
    // Validate both offsets even though the selected convention may not use
    // previous jitter for colour: a malformed depth address is still invalid.
    current_jitter_px.ndc(extent)?;
    previous_jitter_px.ndc(extent)?;
    let current_shift = pixel_shift(current_jitter_px, extent);
    let previous_shift = pixel_shift(previous_jitter_px, extent);
    let current_unjittered = [
        current_raster_uv[0] - current_shift[0],
        current_raster_uv[1] - current_shift[1],
    ];
    let unjittered = [
        current_unjittered[0] + motion_uv[0],
        current_unjittered[1] + motion_uv[1],
    ];
    let raster = [
        unjittered[0] + previous_shift[0],
        unjittered[1] + previous_shift[1],
    ];
    Some(HistoryUv {
        unjittered,
        color: match grid {
            HistoryGrid::ResolvedUnjittered => unjittered,
            HistoryGrid::JitteredScene => raster,
        },
        depth: raster,
    })
}

fn pixel_shift(jitter: JitterOffset, extent: [u32; 2]) -> [f64; 2] {
    [
        jitter.pixels[0] / f64::from(extent[0]),
        jitter.pixels[1] / f64::from(extent[1]),
    ]
}

/// True when a UV addresses a texel centre inside a half-open image domain.
///
/// Bilinear sampling at exactly `1.0` reads the border clamp, which would smear
/// history from the opposite edge; the resolver therefore treats it as outside.
pub fn sampleable(uv: [f64; 2]) -> bool {
    uv.iter()
        .all(|value| value.is_finite() && (0.0..1.0).contains(value))
}

/// Validate projected previous depth against the previous depth buffer.
///
/// `sampled` comes from the previous raster depth texture at the jitter-aware
/// address; `expected` is the current surface point projected through the
/// previous camera. The caller supplies a tolerance derived from its depth
/// format and reprojection error budget. No universal tolerance is assumed here.
pub fn depth_matches(sampled: f64, expected: f64, tolerance: f64) -> bool {
    sampled.is_finite()
        && expected.is_finite()
        && tolerance.is_finite()
        && tolerance >= 0.0
        && (sampled - expected).abs() <= tolerance
}

/// Per-channel YCoCg bounds derived from the current 3x3 neighbourhood.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HistoryBounds {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

/// Derive YCoCg clipping bounds for a current 3x3 neighbourhood.
///
/// The centre is `neighbors[4]`. Returns `None` if any sample is non-finite.
/// A zero standard deviation deliberately produces a zero-width bound, so a
/// flat neighbourhood rejects any history that differs from it.
pub fn neighborhood_bounds(neighbors: &[[f32; 3]; 9], sigma: f32) -> Option<HistoryBounds> {
    if !sigma.is_finite() || sigma < 0.0 {
        return None;
    }
    let mut values = [[0.0f32; 3]; 9];
    for (value, rgb) in values.iter_mut().zip(neighbors) {
        if !rgb.iter().all(|channel| channel.is_finite()) {
            return None;
        }
        *value = rgb_to_ycocg(*rgb);
    }
    let count = 9.0f32;
    let mut mean = [0.0f32; 3];
    for value in values {
        for channel in 0..3 {
            mean[channel] += value[channel];
        }
    }
    for channel in &mut mean {
        *channel /= count;
    }
    let mut deviation = [0.0f32; 3];
    for value in values {
        for channel in 0..3 {
            let delta = value[channel] - mean[channel];
            deviation[channel] += delta * delta;
        }
    }
    let mut bounds = HistoryBounds {
        min: [0.0; 3],
        max: [0.0; 3],
    };
    for channel in 0..3 {
        let spread = (deviation[channel] / count).sqrt() * sigma;
        bounds.min[channel] = mean[channel] - spread;
        bounds.max[channel] = mean[channel] + spread;
    }
    Some(bounds)
}

/// Clamp a history colour into the neighbourhood bounds.
///
/// Clamping happens in YCoCg because the channels are more perceptually
/// decorrelated than pre-exposed RGB. The result is converted back to RGB and
/// negative radiance is clamped to zero. Returns `None` for non-finite input or
/// invalid bounds.
pub fn clip_history(history: [f32; 3], bounds: HistoryBounds) -> Option<[f32; 3]> {
    if !history.iter().all(|channel| channel.is_finite())
        || !bounds
            .min
            .iter()
            .chain(bounds.max.iter())
            .all(|channel| channel.is_finite())
        || (0..3).any(|channel| bounds.min[channel] > bounds.max[channel])
    {
        return None;
    }
    let mut value = rgb_to_ycocg(history);
    for (channel, output) in value.iter_mut().enumerate() {
        *output = output.clamp(bounds.min[channel], bounds.max[channel]);
    }
    let rgb = ycocg_to_rgb(value);
    Some(rgb.map(|channel| channel.max(0.0)))
}

/// History weight derived from the number of already accumulated samples.
///
/// `age == 0` means no usable history and yields zero weight. Age is the count
/// of previous valid frames, so `age == 1` yields the usual 50/50 second frame.
pub fn history_weight(age: u32, minimum: f32) -> f32 {
    if age == 0 {
        return 0.0;
    }
    let minimum = if minimum.is_finite() {
        minimum.clamp(0.0, 1.0)
    } else {
        DEFAULT_MIN_HISTORY_WEIGHT
    };
    (1.0 / (f64::from(age.min(MAX_HISTORY_AGE)) + 1.0) as f32).max(minimum)
}

/// Reason a history candidate was not accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoryRejection {
    EpochReset,
    NoHistory,
    InvalidMotion,
    OutOfBounds,
    NonFiniteHistory,
    DepthDisocclusion,
    InvalidCurrent,
}

/// One previous-frame candidate already sampled by the renderer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HistoryCandidate {
    pub color: [f32; 3],
    pub age: u32,
    pub color_uv: [f64; 2],
    pub depth_uv: [f64; 2],
    pub sampled_depth: f64,
    pub expected_depth: f64,
    pub depth_tolerance: f64,
}

/// Resolve policy values shared by CPU and GPU implementations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvePolicy {
    pub color_sigma: f32,
    pub min_history_weight: f32,
}

impl Default for ResolvePolicy {
    fn default() -> Self {
        Self {
            color_sigma: DEFAULT_COLOR_SIGMA,
            min_history_weight: DEFAULT_MIN_HISTORY_WEIGHT,
        }
    }
}

/// Inputs for one resolved output pixel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolveInput {
    pub current: [f32; 3],
    /// Current 3x3 neighbourhood, centre at index 4.
    pub neighbors: [[f32; 3]; 9],
    pub motion_valid: bool,
    /// False after cut/resize/projection/material/lighting invalidations.
    pub epoch_valid: bool,
    pub history: Option<HistoryCandidate>,
}

/// Resolved pixel and the history state to store for the next frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolveOutput {
    pub color: [f32; 3],
    /// Count of accumulated frames represented by the stored colour.
    pub age: u32,
    /// History weight actually used for this output.
    pub history_weight: f32,
    pub rejection: Option<HistoryRejection>,
}

/// Resolve one pixel through the shared validation and clipping contract.
///
/// Rejection always falls back to the current colour with age one. A valid
/// candidate is clipped against the current neighbourhood before blending and
/// advances the age to `min(age + 1, MAX_HISTORY_AGE)`.
pub fn resolve(input: ResolveInput, policy: ResolvePolicy) -> ResolveOutput {
    if !input.current.iter().all(|channel| channel.is_finite()) {
        return ResolveOutput {
            color: [0.0; 3],
            age: 0,
            history_weight: 0.0,
            rejection: Some(HistoryRejection::InvalidCurrent),
        };
    }
    let current = sanitize_rgb(input.current);
    let fallback = |rejection| ResolveOutput {
        color: current,
        age: 1,
        history_weight: 0.0,
        rejection: Some(rejection),
    };
    if !input.epoch_valid {
        return fallback(HistoryRejection::EpochReset);
    }
    if !input.motion_valid {
        return fallback(HistoryRejection::InvalidMotion);
    }
    let Some(history) = input.history else {
        return fallback(HistoryRejection::NoHistory);
    };
    if history.age == 0 {
        return fallback(HistoryRejection::NoHistory);
    }
    if !sampleable(history.color_uv) || !sampleable(history.depth_uv) {
        return fallback(HistoryRejection::OutOfBounds);
    }
    if !history.color.iter().all(|channel| channel.is_finite()) {
        return fallback(HistoryRejection::NonFiniteHistory);
    }
    if !depth_matches(
        history.sampled_depth,
        history.expected_depth,
        history.depth_tolerance,
    ) {
        return fallback(HistoryRejection::DepthDisocclusion);
    }
    let Some(bounds) = neighborhood_bounds(&input.neighbors, policy.color_sigma) else {
        return fallback(HistoryRejection::InvalidCurrent);
    };
    let Some(clipped) = clip_history(history.color, bounds) else {
        return fallback(HistoryRejection::NonFiniteHistory);
    };
    let weight = history_weight(history.age, policy.min_history_weight);
    if weight <= 0.0 {
        return fallback(HistoryRejection::NoHistory);
    }
    let mut color = [0.0f32; 3];
    for channel in 0..3 {
        color[channel] = current[channel] * (1.0 - weight) + clipped[channel] * weight;
    }
    ResolveOutput {
        color,
        age: history.age.saturating_add(1).min(MAX_HISTORY_AGE),
        history_weight: weight,
        rejection: None,
    }
}

fn sanitize_rgb(rgb: [f32; 3]) -> [f32; 3] {
    rgb.map(|channel| {
        if channel.is_finite() {
            channel.max(0.0)
        } else {
            0.0
        }
    })
}

fn rgb_to_ycocg(rgb: [f32; 3]) -> [f32; 3] {
    let [r, g, b] = rgb;
    [
        0.25 * r + 0.5 * g + 0.25 * b,
        0.5 * r - 0.5 * b,
        -0.25 * r + 0.5 * g - 0.25 * b,
    ]
}

fn ycocg_to_rgb(value: [f32; 3]) -> [f32; 3] {
    let [y, co, cg] = value;
    [y + co - cg, y + cg, y - co - cg]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate(color: [f32; 3], age: u32) -> HistoryCandidate {
        HistoryCandidate {
            color,
            age,
            color_uv: [0.5, 0.5],
            depth_uv: [0.5, 0.5],
            sampled_depth: 0.5,
            expected_depth: 0.5,
            depth_tolerance: 1e-4,
        }
    }

    fn flat(value: [f32; 3]) -> [[f32; 3]; 9] {
        [value; 9]
    }

    #[test]
    fn history_uv_uses_previous_jitter_only_for_raster_data() {
        let extent = [1000, 500];
        let current = JitterOffset::new([0.25, -0.125]);
        let previous = JitterOffset::new([-0.5, 0.5]);
        let uv = [0.5, 0.5];
        let motion = [0.01, -0.02];
        let resolved = history_uv(
            uv,
            motion,
            current,
            previous,
            extent,
            HistoryGrid::ResolvedUnjittered,
        )
        .unwrap();
        let jittered = history_uv(
            uv,
            motion,
            current,
            previous,
            extent,
            HistoryGrid::JitteredScene,
        )
        .unwrap();
        let unjittered = [0.5 - 0.25 / 1000.0 + 0.01, 0.5 + 0.125 / 500.0 - 0.02];
        let raster = [unjittered[0] - 0.5 / 1000.0, unjittered[1] + 0.5 / 500.0];
        for (actual, expected) in resolved.unjittered.iter().zip(unjittered) {
            assert!((actual - expected).abs() < 1e-15);
        }
        assert_eq!(resolved.color, unjittered);
        assert_eq!(jittered.color, raster);
        assert_eq!(resolved.depth, raster);
        assert_eq!(jittered.depth, raster);
    }

    #[test]
    fn stationary_camera_with_changing_jitter_has_no_motion_and_consistent_history() {
        let extent = [1920, 1080];
        let uv = [0.25, 0.75];
        for index in 0..16 {
            let current = JitterOffset::halton(index);
            let previous = JitterOffset::halton(index.wrapping_add(1));
            let resolved = history_uv(
                uv,
                [0.0, 0.0],
                current,
                previous,
                extent,
                HistoryGrid::ResolvedUnjittered,
            )
            .unwrap();
            assert_eq!(
                resolved.color,
                [
                    uv[0] - current.pixels[0] / 1920.0,
                    uv[1] - current.pixels[1] / 1080.0
                ]
            );
            let back = [
                resolved.unjittered[0] - resolved.color[0],
                resolved.unjittered[1] - resolved.color[1],
            ];
            assert!(back.iter().all(|value| value.abs() < 1e-15));
            let jittered_gap = [
                resolved.depth[0] - resolved.unjittered[0],
                resolved.depth[1] - resolved.unjittered[1],
            ];
            let expected_gap = [previous.pixels[0] / 1920.0, previous.pixels[1] / 1080.0];
            assert!(
                jittered_gap
                    .iter()
                    .zip(expected_gap)
                    .all(|(actual, expected)| (actual - expected).abs() < 1e-15)
            );
        }
    }

    #[test]
    fn malformed_lookup_inputs_are_rejected_not_silently_clamped() {
        assert!(
            history_uv(
                [f64::NAN, 0.5],
                [0.0; 2],
                JitterOffset::ZERO,
                JitterOffset::ZERO,
                [10, 10],
                HistoryGrid::ResolvedUnjittered
            )
            .is_none()
        );
        assert!(
            history_uv(
                [0.5, 0.5],
                [0.0; 2],
                JitterOffset::new([0.5001, 0.0]),
                JitterOffset::ZERO,
                [10, 10],
                HistoryGrid::JitteredScene
            )
            .is_none()
        );
        assert!(
            history_uv(
                [0.5, 0.5],
                [0.0; 2],
                JitterOffset::ZERO,
                JitterOffset::ZERO,
                [0, 10],
                HistoryGrid::ResolvedUnjittered
            )
            .is_none()
        );
        assert!(sampleable([0.0, 0.0]));
        assert!(!sampleable([1.0, 0.5]));
        assert!(!sampleable([-0.1, 0.5]));
        assert!(!sampleable([f64::NAN, 0.5]));
    }

    #[test]
    fn valid_history_blends_then_converges_and_increments_age() {
        let current = [1.0, 1.0, 1.0];
        let mut history = candidate([0.5, 0.5, 0.5], 1);
        // Mean 0.5 with enough variance that the 0.5 history is inside the
        // YCoCg bounds; blending then has a predictable weight-only result.
        let mut neighbors = flat([0.375, 0.375, 0.375]);
        neighbors[4] = current;
        let policy = ResolvePolicy::default();
        let mut age = 1;
        for expected_weight in [0.5, 1.0 / 3.0, 0.25] {
            history.age = age;
            let output = resolve(
                ResolveInput {
                    current,
                    neighbors,
                    motion_valid: true,
                    epoch_valid: true,
                    history: Some(history),
                },
                policy,
            );
            assert_eq!(output.rejection, None);
            assert!((output.history_weight - expected_weight).abs() < 1e-6);
            assert_eq!(output.age, age + 1);
            let expected = 0.5 + 0.5 * (1.0 - expected_weight);
            assert!((output.color[0] - expected).abs() < 1e-6);
            age = output.age;
        }
        assert!(history_weight(MAX_HISTORY_AGE, 0.0) >= 1.0 / 65.0);
        assert_eq!(history_weight(0, 0.9), 0.0);
        assert_eq!(history_weight(1, 0.9), 0.9);
    }

    #[test]
    fn outlying_history_is_clipped_before_blending() {
        let current = [1.0, 1.0, 1.0];
        let mut neighbors = flat(current);
        neighbors[0] = [0.8, 0.9, 1.1];
        neighbors[8] = [1.2, 1.1, 0.9];
        let output = resolve(
            ResolveInput {
                current,
                neighbors,
                motion_valid: true,
                epoch_valid: true,
                history: Some(candidate([100.0, 0.0, 0.0], 1)),
            },
            ResolvePolicy::default(),
        );
        assert_eq!(output.rejection, None);
        assert!((output.history_weight - 0.5).abs() < 1e-6);
        assert!(
            output.color.iter().all(|channel| *channel <= 1.25),
            "clipped history must not leak a red ghost: {:?}",
            output.color
        );
        let flat_output = resolve(
            ResolveInput {
                current,
                neighbors: flat(current),
                motion_valid: true,
                epoch_valid: true,
                history: Some(candidate([0.0, 0.0, 0.0], 1)),
            },
            ResolvePolicy::default(),
        );
        assert_eq!(
            flat_output.color, current,
            "zero-variance neighbourhood must reject a different history sample"
        );
    }

    #[test]
    fn every_rejection_uses_current_and_resets_age() {
        let current = [0.2, 0.4, 0.6];
        let base = candidate([0.1, 0.2, 0.3], 3);
        let policy = ResolvePolicy::default();
        let cases = [
            (
                ResolveInput {
                    current,
                    neighbors: flat(current),
                    motion_valid: false,
                    epoch_valid: true,
                    history: Some(base),
                },
                HistoryRejection::InvalidMotion,
            ),
            (
                ResolveInput {
                    current,
                    neighbors: flat(current),
                    motion_valid: true,
                    epoch_valid: false,
                    history: Some(base),
                },
                HistoryRejection::EpochReset,
            ),
            (
                ResolveInput {
                    current,
                    neighbors: flat(current),
                    motion_valid: true,
                    epoch_valid: true,
                    history: None,
                },
                HistoryRejection::NoHistory,
            ),
            (
                ResolveInput {
                    current,
                    neighbors: flat(current),
                    motion_valid: true,
                    epoch_valid: true,
                    history: Some(HistoryCandidate {
                        color_uv: [1.0, 0.5],
                        ..base
                    }),
                },
                HistoryRejection::OutOfBounds,
            ),
            (
                ResolveInput {
                    current,
                    neighbors: flat(current),
                    motion_valid: true,
                    epoch_valid: true,
                    history: Some(HistoryCandidate {
                        sampled_depth: 0.6,
                        ..base
                    }),
                },
                HistoryRejection::DepthDisocclusion,
            ),
            (
                ResolveInput {
                    current,
                    neighbors: flat(current),
                    motion_valid: true,
                    epoch_valid: true,
                    history: Some(HistoryCandidate {
                        color: [f32::NAN, 0.0, 0.0],
                        ..base
                    }),
                },
                HistoryRejection::NonFiniteHistory,
            ),
        ];
        for (input, rejection) in cases {
            let output = resolve(input, policy);
            assert_eq!(output.color, current);
            assert_eq!(output.age, 1);
            assert_eq!(output.history_weight, 0.0);
            assert_eq!(output.rejection, Some(rejection));
        }
        let invalid = resolve(
            ResolveInput {
                current: [f32::NAN, 0.0, 0.0],
                neighbors: flat([0.0; 3]),
                motion_valid: true,
                epoch_valid: true,
                history: Some(base),
            },
            policy,
        );
        assert_eq!(invalid.rejection, Some(HistoryRejection::InvalidCurrent));
        assert_eq!(invalid.age, 0);
    }

    #[test]
    fn depth_tolerance_is_finite_and_explicit() {
        assert!(depth_matches(0.5, 0.50001, 1e-4));
        assert!(!depth_matches(0.5, 0.5002, 1e-4));
        assert!(!depth_matches(f64::NAN, 0.5, 1.0));
        assert!(!depth_matches(0.5, 0.5, f64::NAN));
        assert!(!depth_matches(0.5, 0.5, -1.0));
    }
}
