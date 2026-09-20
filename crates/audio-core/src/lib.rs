//! Backend-neutral acoustic propagation semantics.
//!
//! This crate owns no mixer, samples, Bevy entities, GPU handles, or audio
//! device. It answers physical routing questions: whether an airborne path
//! exists, its propagation delay and Doppler geometry, whether a structural
//! or direct-signal path exists, and when a straight supersonic segment's
//! Mach cone reaches a stationary listener.
//!
//! Presentation volume, samples, HRTF, reverberation and cinematic-space
//! audio stay in the client adapter. See
//! `docs/46_AUDIO_AND_ACOUSTIC_PROPAGATION.md`.

#![forbid(unsafe_code)]

use std::{error::Error, f64::consts::PI, fmt};

use glam::DVec3;
use serde::{Deserialize, Serialize};

const VECTOR_EPSILON_M: f64 = 1.0e-12;
const SPEED_EPSILON_MPS: f64 = 1.0e-12;

/// Player-facing audio policy. Physics always resolves physical paths first;
/// cinematic cues may only be added later by presentation code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AudioPresentationMode {
    Physical,
    Cinematic,
}

/// Local propagation medium sampled by the owning simulation/client layer.
///
/// `density_kg_m3 == 0` is exact vacuum and therefore has no airborne
/// acoustic path. The atmosphere model already owns the vacuum cutoff; this
/// crate does not invent another one.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AcousticMedium {
    pub density_kg_m3: f64,
    pub speed_of_sound_mps: f64,
    /// Amplitude attenuation coefficient in nepers per metre. Intensity uses
    /// `exp(-2 * alpha * distance)`.
    pub absorption_np_per_m: f64,
}

impl AcousticMedium {
    pub const VACUUM: Self = Self {
        density_kg_m3: 0.0,
        speed_of_sound_mps: 0.0,
        absorption_np_per_m: 0.0,
    };

    pub fn gas(
        density_kg_m3: f64,
        speed_of_sound_mps: f64,
        absorption_np_per_m: f64,
    ) -> Result<Self, AudioError> {
        let medium = Self {
            density_kg_m3,
            speed_of_sound_mps,
            absorption_np_per_m,
        };
        medium.validate()?;
        if medium.density_kg_m3 == 0.0 {
            return Err(AudioError::InvalidMedium(
                "gas medium needs positive density; use AcousticMedium::VACUUM".into(),
            ));
        }
        Ok(medium)
    }

    pub fn validate(self) -> Result<(), AudioError> {
        if !self.density_kg_m3.is_finite()
            || !self.speed_of_sound_mps.is_finite()
            || !self.absorption_np_per_m.is_finite()
            || self.density_kg_m3 < 0.0
            || self.speed_of_sound_mps < 0.0
            || self.absorption_np_per_m < 0.0
        {
            return Err(AudioError::InvalidMedium(
                "medium values must be finite and non-negative".into(),
            ));
        }
        if self.density_kg_m3 > 0.0 && self.speed_of_sound_mps <= 0.0 {
            return Err(AudioError::InvalidMedium(
                "non-vacuum medium needs positive sound speed".into(),
            ));
        }
        Ok(())
    }

    pub fn is_vacuum(self) -> bool {
        self.density_kg_m3 == 0.0
    }
}

/// Source/listener kinematics in one common frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AcousticPoint {
    pub position_m: DVec3,
    pub velocity_mps: DVec3,
}

impl AcousticPoint {
    pub const fn stationary(position_m: DVec3) -> Self {
        Self {
            position_m,
            velocity_mps: DVec3::ZERO,
        }
    }

    fn validate(self) -> Result<(), AudioError> {
        if !self.position_m.is_finite() || !self.velocity_mps.is_finite() {
            return Err(AudioError::InvalidKinematics(
                "acoustic point contains a non-finite value".into(),
            ));
        }
        Ok(())
    }
}

/// Geometric airborne path before any sample/mixer mapping.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AirbornePath {
    pub distance_m: f64,
    pub propagation_delay_s: f64,
    /// Ordinary moving-source/listener Doppler ratio when the classical
    /// denominator remains valid. Supersonic/shock cases return `None` and
    /// are handled by pressure-wave logic instead.
    pub doppler_ratio: Option<f64>,
}

/// Resolve direct airborne propagation through one locally uniform medium.
///
/// Occlusion and multi-volume routing are intentionally separate layers. This
/// primitive establishes the invariant that exact vacuum has no airborne
/// acoustic path.
pub fn resolve_airborne_path(
    source: AcousticPoint,
    listener: AcousticPoint,
    medium: AcousticMedium,
) -> Result<Option<AirbornePath>, AudioError> {
    source.validate()?;
    listener.validate()?;
    medium.validate()?;
    if medium.is_vacuum() {
        return Ok(None);
    }

    let delta = listener.position_m - source.position_m;
    let distance_m = delta.length();
    if !distance_m.is_finite() {
        return Err(AudioError::InvalidKinematics(
            "source/listener distance is non-finite".into(),
        ));
    }
    if distance_m <= VECTOR_EPSILON_M {
        return Ok(Some(AirbornePath {
            distance_m: 0.0,
            propagation_delay_s: 0.0,
            doppler_ratio: Some(1.0),
        }));
    }

    let direction = delta / distance_m;
    let source_radial_mps = source.velocity_mps.dot(direction);
    let listener_radial_mps = listener.velocity_mps.dot(direction);
    let denominator = medium.speed_of_sound_mps - source_radial_mps;
    let numerator = medium.speed_of_sound_mps - listener_radial_mps;
    let doppler_ratio = if denominator > SPEED_EPSILON_MPS && numerator > SPEED_EPSILON_MPS {
        let ratio = numerator / denominator;
        ratio.is_finite().then_some(ratio)
    } else {
        None
    };

    Ok(Some(AirbornePath {
        distance_m,
        propagation_delay_s: distance_m / medium.speed_of_sound_mps,
        doppler_ratio,
    }))
}

/// Spherical-spreading intensity for a calibrated acoustic source.
///
/// `reference_radius_m` is the near-field boundary at which the far-field
/// point-source approximation begins; it prevents a singularity without
/// inventing a gameplay distance clamp.
pub fn airborne_intensity_w_m2(
    acoustic_power_w: f64,
    reference_radius_m: f64,
    medium: AcousticMedium,
    path: AirbornePath,
) -> Result<f64, AudioError> {
    medium.validate()?;
    if medium.is_vacuum() {
        return Err(AudioError::InvalidMedium(
            "airborne intensity is undefined in vacuum".into(),
        ));
    }
    if !acoustic_power_w.is_finite()
        || acoustic_power_w < 0.0
        || !reference_radius_m.is_finite()
        || reference_radius_m <= 0.0
        || !path.distance_m.is_finite()
        || path.distance_m < 0.0
    {
        return Err(AudioError::InvalidSource(
            "source power/radius and path distance must be finite and physical".into(),
        ));
    }

    let radius_m = path.distance_m.max(reference_radius_m);
    let spreading = acoustic_power_w / (4.0 * PI * radius_m * radius_m);
    let absorption = (-2.0 * medium.absorption_np_per_m * path.distance_m).exp();
    Ok(spreading * absorption)
}

/// A structure-borne path exists when source and listener share the same
/// connected structural cluster. Attenuation is deliberately not guessed
/// here; the future structural graph owns path length/material interfaces.
pub fn structure_path_exists(source_cluster: Option<u64>, listener_cluster: Option<u64>) -> bool {
    source_cluster.is_some() && source_cluster == listener_cluster
}

/// Electrical/radio/speaker signal connectivity is independent of atmosphere.
pub fn direct_signal_path_exists(source_bus: Option<u64>, listener_bus: Option<u64>) -> bool {
    source_bus.is_some() && source_bus == listener_bus
}

/// Constant-velocity supersonic source segment used by the first boom slice.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SupersonicSegment {
    pub start_time_s: f64,
    pub duration_s: f64,
    pub start_position_m: DVec3,
    pub velocity_mps: DVec3,
}

impl SupersonicSegment {
    fn validate(self) -> Result<(), AudioError> {
        if !self.start_time_s.is_finite()
            || !self.duration_s.is_finite()
            || self.duration_s <= 0.0
            || !self.start_position_m.is_finite()
            || !self.velocity_mps.is_finite()
        {
            return Err(AudioError::InvalidKinematics(
                "supersonic segment must contain finite values and positive duration".into(),
            ));
        }
        Ok(())
    }
}

/// Tangency of a straight Mach cone with one stationary listener.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SonicBoomArrival {
    pub emission_time_s: f64,
    pub arrival_time_s: f64,
    pub emission_position_m: DVec3,
    pub propagation_distance_m: f64,
}

/// Find the pressure-wave arrival generated by a straight constant-velocity
/// supersonic segment in a locally uniform medium.
///
/// The minimum arrival-time condition is the Mach-cone tangency:
/// the source-to-listener line has longitudinal projection `c / v`.
/// Returning `None` means this finite segment does not sweep its Mach cone
/// across the listener. The listener is stationary for this first slice.
pub fn sonic_boom_arrival(
    segment: SupersonicSegment,
    listener_position_m: DVec3,
    medium: AcousticMedium,
) -> Result<Option<SonicBoomArrival>, AudioError> {
    segment.validate()?;
    medium.validate()?;
    if medium.is_vacuum() {
        return Ok(None);
    }
    if !listener_position_m.is_finite() {
        return Err(AudioError::InvalidKinematics(
            "boom listener position is non-finite".into(),
        ));
    }

    let speed_mps = segment.velocity_mps.length();
    if !speed_mps.is_finite() || speed_mps <= medium.speed_of_sound_mps {
        return Ok(None);
    }

    let direction = segment.velocity_mps / speed_mps;
    let to_listener = listener_position_m - segment.start_position_m;
    let longitudinal_m = to_listener.dot(direction);
    let lateral_sq_m2 = (to_listener.length_squared() - longitudinal_m * longitudinal_m).max(0.0);
    let lateral_m = lateral_sq_m2.sqrt();
    let mach_denominator =
        (speed_mps * speed_mps - medium.speed_of_sound_mps * medium.speed_of_sound_mps).sqrt();
    let longitudinal_at_emission_m = medium.speed_of_sound_mps * lateral_m / mach_denominator;
    let emission_offset_s = (longitudinal_m - longitudinal_at_emission_m) / speed_mps;

    if emission_offset_s < 0.0 || emission_offset_s > segment.duration_s {
        return Ok(None);
    }

    let emission_position_m = segment.start_position_m + segment.velocity_mps * emission_offset_s;
    let propagation_distance_m = (listener_position_m - emission_position_m).length();
    let emission_time_s = segment.start_time_s + emission_offset_s;
    let arrival_time_s = emission_time_s + propagation_distance_m / medium.speed_of_sound_mps;

    Ok(Some(SonicBoomArrival {
        emission_time_s,
        arrival_time_s,
        emission_position_m,
        propagation_distance_m,
    }))
}

#[derive(Debug, Clone, PartialEq)]
pub enum AudioError {
    InvalidMedium(String),
    InvalidKinematics(String),
    InvalidSource(String),
}

impl fmt::Display for AudioError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidMedium(message) => write!(formatter, "invalid acoustic medium: {message}"),
            Self::InvalidKinematics(message) => {
                write!(formatter, "invalid acoustic kinematics: {message}")
            }
            Self::InvalidSource(message) => write!(formatter, "invalid acoustic source: {message}"),
        }
    }
}

impl Error for AudioError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn air() -> AcousticMedium {
        AcousticMedium::gas(1.225, 340.0, 0.0).unwrap()
    }

    #[test]
    fn exterior_vacuum_has_no_airborne_engine_path() {
        let source = AcousticPoint::stationary(DVec3::ZERO);
        let listener = AcousticPoint::stationary(DVec3::new(20.0, 0.0, 0.0));
        assert_eq!(
            resolve_airborne_path(source, listener, AcousticMedium::VACUUM).unwrap(),
            None
        );
    }

    #[test]
    fn cabin_keeps_structure_path_in_vacuum() {
        assert!(structure_path_exists(Some(7), Some(7)));
        assert!(!structure_path_exists(Some(7), Some(8)));
        assert!(!structure_path_exists(Some(7), None));
    }

    #[test]
    fn gpws_signal_path_does_not_depend_on_atmosphere() {
        assert!(AcousticMedium::VACUUM.is_vacuum());
        assert!(direct_signal_path_exists(Some(3), Some(3)));
        assert!(!direct_signal_path_exists(Some(3), Some(4)));
    }

    #[test]
    fn airborne_path_has_finite_delay_and_inverse_square_intensity() {
        let source = AcousticPoint::stationary(DVec3::ZERO);
        let near = AcousticPoint::stationary(DVec3::new(10.0, 0.0, 0.0));
        let far = AcousticPoint::stationary(DVec3::new(20.0, 0.0, 0.0));
        let near_path = resolve_airborne_path(source, near, air()).unwrap().unwrap();
        let far_path = resolve_airborne_path(source, far, air()).unwrap().unwrap();
        assert!((near_path.propagation_delay_s - 10.0 / 340.0).abs() < 1.0e-12);
        let near_i = airborne_intensity_w_m2(1_000.0, 1.0, air(), near_path).unwrap();
        let far_i = airborne_intensity_w_m2(1_000.0, 1.0, air(), far_path).unwrap();
        assert!((near_i / far_i - 4.0).abs() < 1.0e-12);
    }

    #[test]
    fn boom_arrives_after_supersonic_source_has_visibly_passed() {
        let segment = SupersonicSegment {
            start_time_s: 0.0,
            duration_s: 4.0,
            start_position_m: DVec3::new(-1_000.0, 300.0, 0.0),
            velocity_mps: DVec3::new(500.0, 0.0, 0.0),
        };
        let arrival = sonic_boom_arrival(segment, DVec3::ZERO, air())
            .unwrap()
            .expect("Mach cone should sweep the listener");
        let visible_pass_time_s = 1_000.0 / 500.0;
        assert!(arrival.emission_time_s < visible_pass_time_s);
        assert!(arrival.arrival_time_s > visible_pass_time_s);
        assert!(arrival.arrival_time_s > arrival.emission_time_s);
    }

    #[test]
    fn subsonic_segment_has_no_sonic_boom() {
        let segment = SupersonicSegment {
            start_time_s: 0.0,
            duration_s: 10.0,
            start_position_m: DVec3::new(-1_000.0, 100.0, 0.0),
            velocity_mps: DVec3::new(300.0, 0.0, 0.0),
        };
        assert_eq!(
            sonic_boom_arrival(segment, DVec3::ZERO, air()).unwrap(),
            None
        );
    }
}
