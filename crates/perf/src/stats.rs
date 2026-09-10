//! p50 / p95 / p99 / max over a rolling window.
//!
//! Percentiles matter more than averages for terrain generation and streaming:
//! most frames at 8 ms with occasional 70 ms stalls is not an 8 ms experience.

use serde::{Deserialize, Serialize};

/// Summary of one duration series in the current window (seconds).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FrameStats {
    pub current: f64,
    pub p50: f64,
    pub p95: f64,
    pub p99: f64,
    pub max: f64,
    pub count: usize,
}

/// Nearest-rank percentile over an already sorted ascending slice.
/// Returns 0.0 for empty input rather than NaN so overlays stay stable.
pub fn percentile_sorted(sorted: &[f64], quantile: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let q = quantile.clamp(0.0, 1.0);
    if q <= 0.0 {
        return sorted[0];
    }
    if q >= 1.0 {
        return sorted[sorted.len() - 1];
    }
    let rank = (q * sorted.len() as f64).ceil() as usize;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

/// Summarize wall/CPU/scope durations. `values` are seconds; the last element
/// is treated as `current`.
pub fn summarize(values: &[f64]) -> Option<FrameStats> {
    if values.is_empty() {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.total_cmp(b));
    Some(FrameStats {
        current: values[values.len() - 1],
        p50: percentile_sorted(&sorted, 0.50),
        p95: percentile_sorted(&sorted, 0.95),
        p99: percentile_sorted(&sorted, 0.99),
        max: sorted[sorted.len() - 1],
        count: values.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentiles_match_nearest_rank() {
        let sorted = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        assert_eq!(percentile_sorted(&sorted, 0.50), 5.0);
        assert_eq!(percentile_sorted(&sorted, 0.95), 10.0);
        assert_eq!(percentile_sorted(&sorted, 0.99), 10.0);
        assert_eq!(percentile_sorted(&sorted, 0.0), 1.0);
        assert_eq!(percentile_sorted(&sorted, 1.0), 10.0);
    }

    #[test]
    fn long_tail_hitch_visible_in_p99_but_not_median() {
        let mut values = vec![0.008; 90];
        values.extend(std::iter::repeat_n(0.070, 10));
        let stats = summarize(&values).unwrap();
        assert!((stats.p50 - 0.008).abs() < 1e-12);
        assert!((stats.p95 - 0.070).abs() < 1e-12);
        assert!((stats.p99 - 0.070).abs() < 1e-12);
        assert!((stats.max - 0.070).abs() < 1e-12);
    }

    #[test]
    fn empty_window_has_no_stats() {
        assert!(summarize(&[]).is_none());
        assert_eq!(percentile_sorted(&[], 0.95), 0.0);
    }
}
