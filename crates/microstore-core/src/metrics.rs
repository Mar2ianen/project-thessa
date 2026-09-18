//! Declared quality metrics (doc §13): max absolute, RMS, and mean
//! absolute error between an original and a decoded scalar field.

use crate::codec::ScalarField;

/// Error summary over all texels of the true extent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ErrorStats {
    /// Worst per-texel `|decoded - original|` in `u8` levels.
    pub max_abs: f64,
    /// Root-mean-square error in `u8` levels.
    pub rms: f64,
    /// Mean absolute error in `u8` levels.
    pub mean_abs: f64,
    /// Compared texel count.
    pub texels: u64,
}

/// Measure decode error. Panics on extent mismatch: comparing fields of
/// different shape is a harness bug, not a codec result.
pub fn measure(original: &ScalarField, decoded: &ScalarField) -> ErrorStats {
    assert_eq!(
        (original.width, original.height),
        (decoded.width, decoded.height),
        "error metrics need identical extents"
    );
    let mut worst = 0u32;
    let mut sum_abs = 0u64;
    let mut sum_sq = 0u64;
    for (a, b) in original.data.iter().zip(decoded.data.iter()) {
        let d = a.abs_diff(*b) as u64;
        worst = worst.max(d as u32);
        sum_abs += d;
        sum_sq += d * d;
    }
    let n = original.data.len() as f64;
    ErrorStats {
        max_abs: worst as f64,
        rms: (sum_sq as f64 / n).sqrt(),
        mean_abs: sum_abs as f64 / n,
        texels: original.data.len() as u64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codec::ScalarField;

    fn field(data: &[u8]) -> ScalarField {
        let n = data.len() as u32;
        ScalarField::new(n, 1, data.to_vec()).expect("test field")
    }

    #[test]
    fn identical_fields_have_zero_error() {
        let a = field(&[0, 1, 2, 250, 255]);
        let stats = measure(&a, &a);
        assert_eq!(
            stats,
            ErrorStats {
                max_abs: 0.0,
                rms: 0.0,
                mean_abs: 0.0,
                texels: 5
            }
        );
    }

    #[test]
    fn known_values_match_hand_computation() {
        // |diffs| = 1, 2, 3: max 3, mean 2, rms sqrt(14/3).
        let a = field(&[0, 10, 20]);
        let b = field(&[1, 12, 17]);
        let stats = measure(&a, &b);
        assert_eq!(stats.max_abs, 3.0);
        assert_eq!(stats.mean_abs, 2.0);
        assert!((stats.rms - (14.0f64 / 3.0).sqrt()).abs() < 1e-12);
        assert_eq!(stats.texels, 3);
    }

    #[test]
    fn worst_case_is_255() {
        let a = field(&[0]);
        let b = field(&[255]);
        let stats = measure(&a, &b);
        assert_eq!(stats.max_abs, 255.0);
        assert_eq!(stats.rms, 255.0);
        assert_eq!(stats.mean_abs, 255.0);
    }

    #[test]
    #[should_panic(expected = "identical extents")]
    fn mismatched_extents_panic() {
        let a = field(&[1, 2]);
        let b = ScalarField::new(1, 2, vec![1, 2]).unwrap();
        let _ = measure(&a, &b);
    }
}
