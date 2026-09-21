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
//! (left) surfaces negate the roll AND yaw gains at mix time
//! (symmetric-pair rule: ailerons, rudders, ruddervators);
//! pitch/flap/airbrake gains stay put. The compiler records the parent
//! chain so the mixer can address it.
//!
//! One-sided devices (spoiler, airbrake, slat with minimum exactly 0)
//! are presets since the sim-core limit model parks negative commands at
//! zero; the flap preset keeps a documented −1 deg reflex shim.

use serde::{Deserialize, Serialize};

use crate::{ControlRegion, ControlRegionKind, SurfaceError};

/// Channel mixing gains for one control region. The runtime command is
/// `pitch*k_pitch + roll*k_roll + yaw*k_yaw + flap*k_flap +
/// airbrake*k_airbrake`, saturated to `[-1, 1]` by [`mix_command`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlMixing {
    /// Pitch channel gain (elevator, elevon).
    pub pitch: f64,
    /// Roll channel gain (aileron, elevon, flaperon; negated on mirrors).
    pub roll: f64,
    /// Yaw channel gain (rudder).
    pub yaw: f64,
    /// Flap deployment channel gain (flap, flaperon, slat).
    pub flap: f64,
    /// Airbrake channel gain (spoiler panels, airbrake).
    pub airbrake: f64,
}

/// Pilot/trim channel inputs, each in `[-1, 1]` (flap/airbrake `0..=1`
/// typical).
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
    /// Airbrake input, 1 fully deployed.
    pub airbrake: f64,
}

impl ControlChannels {
    /// Neutral sticks, clean wing.
    pub fn neutral() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 0.0,
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
        ("airbrake", channels.airbrake),
    ] {
        assert!(
            value.is_finite() && (-1.0..=1.0).contains(&value),
            "channel {label} must be in [-1, 1]"
        );
    }
    (mixing.pitch * channels.pitch
        + mixing.roll * channels.roll
        + mixing.yaw * channels.yaw
        + mixing.flap * channels.flap
        + mixing.airbrake * channels.airbrake)
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 0.0,
            roll: 1.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 0.0,
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 1.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 0.0,
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 0.0,
            roll: 0.0,
            yaw: 1.0,
            flap: 0.0,
            airbrake: 0.0,
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 1.0,
            roll: 1.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 0.0,
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 0.0,
            roll: 1.0,
            yaw: 0.0,
            flap: 1.0,
            airbrake: 0.0,
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 1.0,
            airbrake: 0.0,
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 1.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 0.0,
        },
    )
}

/// Anti-servo tab: nested tab geared to move against the parent command
/// (negative pitch gain), used on stabilators and weight-shift-correct
/// tails. Sizing and gearing ratios are airframe data; the sign is the
/// preset.
pub fn anti_servo_tab(
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: -1.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 0.0,
        },
    )
}

/// Ruddervator: V-tail (or inverted-V) trailing-edge region driven by
/// pitch plus yaw. Each V half authors the same preset; the mirrored
/// half negates roll and yaw gains at mix time (symmetric-pair rule),
/// pitch/flap/airbrake gains stay put.
pub fn ruddervator(
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
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 1.0,
            roll: 0.0,
            yaw: 1.0,
            flap: 0.0,
            airbrake: 0.0,
        },
    )
}

/// Spoiler: one-sided mid-chord panel (0 to +60 deg) for roll spoilers
/// and glide-path control. Negative commands park at zero through the
/// one-sided limit model.
pub fn spoiler(
    name: impl Into<String>,
    span: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        (0.35, 0.75),
        0.35,
        0.0,
        60.0_f64.to_radians(),
        None,
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 0.0,
            roll: 1.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 1.0,
        },
    )
}

/// Airbrake / speedbrake panel: one-sided symmetric device on the
/// dedicated airbrake channel. Mount pairs on wings or fuselage sides;
/// symmetric deployment is a mixer pairing rule, not geometry.
pub fn airbrake(
    name: impl Into<String>,
    span: (f64, f64),
    chord: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        chord,
        chord.0,
        0.0,
        55.0_f64.to_radians(),
        None,
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 1.0,
        },
    )
}

/// Slat: leading-edge high-lift region on the flap channel. Slat motion
/// is not a pure hinge rotation (translation presets are future
/// kinematics); the region-on-parent authoring and deployment channel
/// are the preset, matching the doc's slat clause.
pub fn slat(
    name: impl Into<String>,
    span: (f64, f64),
) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        span,
        (0.0, 0.15),
        0.0,
        0.0,
        25.0_f64.to_radians(),
        None,
        ControlRegionKind::TrailingEdgeDevice,
        ControlMixing {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 1.0,
            airbrake: 0.0,
        },
    )
}

/// Stabilator / all-moving tail: a region covering the full span and
/// chord, marked [`ControlRegionKind::AllMovingSurface`] so the runtime
/// rotates the whole surface instead of deflecting trailing-edge
/// panels. Pivot placement and actuator rates are runtime data; the
/// quarter-chord hinge reference here is the conventional default.
pub fn stabilator(name: impl Into<String>) -> Result<(ControlRegion, ControlMixing), SurfaceError> {
    region_with_mix(
        name,
        (0.0, 1.0),
        (0.0, 1.0),
        0.25,
        -20.0_f64.to_radians(),
        20.0_f64.to_radians(),
        None,
        ControlRegionKind::AllMovingSurface,
        ControlMixing {
            pitch: 1.0,
            roll: 0.0,
            yaw: 0.0,
            flap: 0.0,
            airbrake: 0.0,
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
    kind: ControlRegionKind,
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
        kind,
    };
    // Standalone check covers bounds, hinge placement, and limits.
    // Nesting containment against the real parent is a surface-level
    // property, checked at surface validation with siblings present.
    region.validate(&[])?;
    region.parent = parent;
    Ok((region, mixing))
}
