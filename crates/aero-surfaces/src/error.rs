//! Compiler error type.

use std::fmt;

/// Failure to author or compile a procedural surface.
///
/// Every variant carries the offending value or name so a hangar UI can
/// point at the bad field instead of failing opaquely.
#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceError {
    /// A station list is empty, unsorted, out of `[0, 1]`, or degenerate.
    InvalidStations(String),
    /// A chord is non-positive or non-finite at some station.
    InvalidChord(String),
    /// Bend offsets are non-finite.
    InvalidBend(String),
    /// Section data (incidence, thickness) is out of range or non-finite.
    InvalidSection(String),
    /// A control region has bad bounds, hinge placement, limits, or nesting.
    InvalidControlRegion(String),
    /// A fold joint has a bad station, axis, angle, or limit.
    InvalidFoldJoint(String),
    /// Surface-level assembly (span, mount, duplicate names) is invalid.
    InvalidSurface(String),
    /// Compilation options (tolerances, depth, mechanism state) are invalid.
    InvalidOptions(String),
    /// The underlying solver primitive rejected derived geometry. This is a
    /// compiler bug unless the message names an authoring value that slipped
    /// validation; it is surfaced rather than panicked on.
    PanelRejected(String),
}

impl fmt::Display for SurfaceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidStations(message)
            | Self::InvalidChord(message)
            | Self::InvalidBend(message)
            | Self::InvalidSection(message)
            | Self::InvalidControlRegion(message)
            | Self::InvalidFoldJoint(message)
            | Self::InvalidSurface(message)
            | Self::InvalidOptions(message)
            | Self::PanelRejected(message) => write!(f, "{message}"),
        }
    }
}

impl std::error::Error for SurfaceError {}
