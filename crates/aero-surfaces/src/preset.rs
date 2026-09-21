//! User-facing control presets over generic region geometry.
//!
//! An aileron is not an aerodynamic primitive: it is a trailing-edge
//! region with conventional hinge placement, limits, and a roll mixing.
//! An elevon is the same region shape with a pitch-plus-roll mix; a
//! flaperon mixes flap deployment with roll. This module builds those
//! presets as [`ControlRegion`] plus [`ControlMixing`] data. Geometry and
//! limits compile today; mixing gains are data for the future FBW mixer
//! (evaluated here only by the pure [`mix_command`] helper, which pins
//! the documented formulas).
//!
//! Sign conventions (right-hand surface, documented, not enforced):
//! positive pitch/roll/yaw/flap channel values drive trailing-edge-down
//! on a right wing for pitch/flap, right-wing-up for positive roll, and
//! trailing-edge-right for positive yaw on a vertical tail. Mirrored
//! (left) surfaces negate the roll gain at mix time; the compiler records
//! the chain so the mixer can address it.
//!
//! One-sided devices (spoiler, airbrake, slat) are deliberately NOT
//! presets yet: `ControlSurfaceDefinition` requires `min < 0 < max`, so a
//! pure one-way device needs a sim-core limit-model extension first. The
//! flap preset carries a −1 deg reflex shim for the same reason,
//! documented at the constructor; the mixer never commands it.

use serde::{Deserialize, Serialize};

use crate::{ControlRegion, SurfaceError};

/// Channel mixing gains for one control region. The runtime command is
/// `pitch*k_pitch + roll*k_roll + yaw*k_yaw + flap*k_flap`, saturated to
/// `[-1, 1]` by [`mix_command`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlMixing {
    /// Pitch channel gain (elevator, elevon).
    pub pitch: f64,
    /// Roll channel gain (aileron, elevon, flaperon; negated on mirrors).
    pub roll: f64,
    /// Yaw channel gain (rudder).
    pub yaw: f64,
    /// Flap deployment channel gain (flap, flaperon).
    pub flap: f64,
}

/// Pilot/trim channel inputs, each in `[-1, 1]` (flap `0..=1` typical).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlChannels {
    /// Pitch input, +1 trailing-edge-down on a right wing.
    pub pitch: f64,
    /// Roll input, +1 right-wing-up.
    pub roll: f64,
    /// Yaw input, +1 trailing-edge-right on a vertical tail.
    pub yaw: f64,
    /// Flap deployment input, 1 fully deployed.
    pub flap: f64,
}

impl ControlChannels {
    /// Neutral sticks, clean wing.
    pub fn neutral() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
        }
    }
}

/// Evaluate the documented mixing formulas with `[-1, 1]` saturation:
/// `elevon = pitch + roll`, `flaperon = flap + roll`, plain surfaces take
/// their single channel. Pure function: pins preset semantics without a
/// runtime mixer.
pub fn mix_command(mixing: ControlMixing, channels: ControlChannels) -> f64 {
    for (label, value) in [
        ("pitch", channels.pitch),
        ("roll", channels.roll),
        ("yaw", channels.yaw),
        ("flap", channels.flap),
    ] {
        assert!(
            value.is_finite() && (-1.0..=1.0).contains(&value),
            "channel {label} must be in [-1, 1]"
        );
    }
    (mixing.pitch * channels.pitch
        + mixing.roll * channels.roll
        + mixing.yaw * channels.yaw
        + mixing.flap * channels.flap)
        .clamp(-1.0, 1.0)
}

/// Aileron: trailing-edge region with roll mixing.
pub fn aileron(
    name: impl Into<String>,
    span: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        (0.25, 1.0),
        0.25,
        -25.0_f64.to_radians(),
        25.0_f64.to_radians(),
        None,
        ControlMixing {
            pitch: 0.0,
            roll: 1.0,
            yaw: 0.0,
            flap: 0.0,
        },
    )
}

/// Elevator: trailing-edge region with pitch mixing.
pub fn elevator(
    name: impl Into<String>,
    span: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        (0.2, 1.0),
        0.2,
        -25.0_f64.to_radians(),
        25.0_f64.to_radians(),
        None,
        ControlMixing {
            pitch: 1.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
        },
    )
}

/// Rudder: trailing-edge region on a vertical tail with yaw mixing.
/// The compiler is orientation-agnostic; mount the surface vertically.
pub fn rudder(
    name: impl Into<String>,
    span: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        (0.3, 1.0),
        0.3,
        -30.0_f64.to_radians(),
        30.0_f64.to_radians(),
        None,
        ControlMixing {
            pitch: 0.0,
            roll: 0.0,
            yaw: 1.0,
            flap: 0.0,
        },
    )
}

/// Elevon: elevator-shaped region driven by pitch plus roll.
pub fn elevon(
    name: impl Into<String>,
    span: (f64, f64),
    chord: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        chord,
        chord.0,
        -25.0_f64.to_radians(),
        25.0_f64.to_radians(),
        None,
        ControlMixing {
            pitch: 1.0,
            roll: 1.0,
            yaw: 0.0,
            flap: 0.0,
        },
    )
}

/// Flaperon: aileron-shaped region driven by flap deployment plus roll.
pub fn flaperon(
    name: impl Into<String>,
    span: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        (0.25, 1.0),
        0.25,
        -5.0_f64.to_radians(),
        25.0_f64.to_radians(),
        None,
        ControlMixing {
            pitch: 0.0,
            roll: 1.0,
            yaw: 0.0,
            flap: 1.0,
        },
    )
}

/// Flap: high-lift trailing-edge region on the flap channel. The −1 deg
/// minimum is a validation-compat reflex shim (one-sided limits need a
/// sim-core extension); the mixer never commands negative flap.
pub fn flap(
    name: impl Into<String>,
    span: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        (0.15, 1.0),
        0.15,
        -1.0_f64.to_radians(),
        35.0_f64.to_radians(),
        None,
        ControlMixing {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 1.0,
        },
    )
}

/// Trim tab: small nested region inside a parent, driven by the trim
/// channel modeled here as pitch input (dedicated trim wiring is FBW
/// future work).
pub fn trim_tab(
    name: impl Into<String>,
    span: (f64, f64),
    chord: (f64, f64),
    parent: usize,
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        chord,
        chord.0,
        -15.0_f64.to_radians(),
        15.0_f64.to_radians(),
        Some(parent),
        ControlMixing {
            pitch: 1.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
        },
    )
}

#[allow(clippy::too_many_arguments)]
fn region_with_mix(
    name: impl Into<String>,
    span: (f64, f64),
    chord: (f64, f64),
    hinge_u: f64,
    min_deflection_rad: f64,
    max_deflection_rad: f64,
    parent: Option<usize>,
    mixing: ControlMixing,
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    let mut region = ControlRegion {
        name: name.into(),
        span,
        chord,
        hinge_u,
        min_deflection_rad,
        max_deflection_rad,
        parent: None,
    };
    // Standalone check covers bounds, hinge placement, and limits.
    // Nesting containment against the real parent is a surface-level
    // property, checked at surface validation with siblings present.
    region.validate(&[])?;
    region.parent = parent;
    Ok((region, mixing))
}
