//! Validation errors and NaN-closed guards shared by the propulsion backend.

use std::{error::Error, fmt};

/// Propulsion authoring/compile/runtime errors.
#[derive(Debug, Clone, PartialEq)]
pub enum PropulsionError {
    /// Authoring value outside its physical/engineering domain.
    InvalidSpec(String),
    /// Cycle/material/cooling combination cannot support the design point.
    UnsupportedCombination(String),
    /// Runtime command the engine cannot execute (solid throttle, burnout).
    InvalidCommand(String),
}

impl fmt::Display for PropulsionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSpec(message) => write!(formatter, "invalid engine spec: {message}"),
            Self::UnsupportedCombination(message) => {
                write!(formatter, "unsupported engine combination: {message}")
            }
            Self::InvalidCommand(message) => {
                write!(formatter, "invalid engine command: {message}")
            }
        }
    }
}

impl Error for PropulsionError {}

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub(crate) fn require_positive(value: f64, name: &str) -> Result<f64, PropulsionError> {
    if !(value > 0.0) {
        return Err(PropulsionError::InvalidSpec(format!(
            "{name} must be finite and > 0"
        )));
    }
    Ok(value)
}

#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub(crate) fn require_non_negative(value: f64, name: &str) -> Result<f64, PropulsionError> {
    if !(value >= 0.0) {
        return Err(PropulsionError::InvalidSpec(format!(
            "{name} must be finite and >= 0"
        )));
    }
    Ok(value)
}

pub(crate) fn require_unit_interval(value: f64, name: &str) -> Result<f64, PropulsionError> {
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err(PropulsionError::InvalidSpec(format!(
            "{name} must be finite in [0, 1]"
        )));
    }
    Ok(value)
}
