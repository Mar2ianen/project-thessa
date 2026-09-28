//! Procedural fuselage authoring failure modes.
//!
//! Every variant fails closed with a named cause: the hangar compiler never
//! guesses geometry, and the flight path never sees an authoring object.

use std::{error::Error, fmt};

/// Hangar-side fuselage authoring or compilation failure.
#[derive(Debug, Clone, PartialEq)]
pub enum FuselageError {
    /// Authoring stations or body topology are inconsistent.
    InvalidBody(String),
    /// A cross-section station is inconsistent.
    InvalidSection(String),
    /// An interior region or port is inconsistent.
    InvalidInterior(String),
    /// Compile options are inconsistent.
    InvalidOptions(String),
    /// A derived zone or panel violates solver input rules.
    PanelRejected(String),
    /// A contact part violates collision input rules.
    InvalidCollision(String),
}

impl fmt::Display for FuselageError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidBody(message) => write!(formatter, "invalid fuselage body: {message}"),
            Self::InvalidSection(message) => {
                write!(formatter, "invalid fuselage section: {message}")
            }
            Self::InvalidInterior(message) => {
                write!(formatter, "invalid fuselage interior: {message}")
            }
            Self::InvalidOptions(message) => {
                write!(formatter, "invalid fuselage compile options: {message}")
            }
            Self::PanelRejected(message) => {
                write!(formatter, "fuselage panel rejected: {message}")
            }
            Self::InvalidCollision(message) => {
                write!(formatter, "invalid fuselage contact geometry: {message}")
            }
        }
    }
}

impl Error for FuselageError {}
