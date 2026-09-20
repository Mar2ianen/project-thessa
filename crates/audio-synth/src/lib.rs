//! Procedural audio DSP for Project Thessa.
//!
//! This crate deliberately knows nothing about Bevy, rodio, platform audio
//! devices, ECS entities, or sample assets. It turns slowly changing physical
//! source controls into PCM samples. A client/backend adapter decides how
//! those samples reach an audio device.

#![forbid(unsafe_code)]

use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicU32, Ordering},
};

pub const DEFAULT_SAMPLE_RATE_HZ: u32 = 48_000;

fn sanitize_unit(value: f32) -> f32 {
    if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    }
}

fn store_f32(slot: &AtomicU32, value: f32) {
    slot.store(value.to_bits(), Ordering::Relaxed);
}

fn load_f32(slot: &AtomicU32) -> f32 {
    f32::from_bits(slot.load(Ordering::Relaxed))
}

/// Live engine controls shared between the simulation/presentation thread and
/// the audio decoder.
///
/// These are semantic mix controls, not backend voice handles. The audio
/// thread reads atomics and smooths them locally to avoid clicks.
#[derive(Debug)]
pub struct EngineSynthControl {
    active: AtomicBool,
    throttle: AtomicU32,
    airborne_gain: AtomicU32,
    structure_gain: AtomicU32,
}

impl Default for EngineSynthControl {
    fn default() -> Self {
        Self {
            active: AtomicBool::new(false),
            throttle: AtomicU32::new(0.0f32.to_bits()),
            airborne_gain: AtomicU32::new(0.0f32.to_bits()),
            structure_gain: AtomicU32::new(0.0f32.to_bits()),
        }
    }
}

impl EngineSynthControl {
    pub fn set_active(&self, active: bool) {
        self.active.store(active, Ordering::Relaxed);
    }

    pub fn set_throttle(&self, throttle: f32) {
        store_f32(&self.throttle, sanitize_unit(throttle));
    }

    pub fn set_airborne_gain(&self, gain: f32) {
        store_f32(&self.airborne_gain, sanitize_unit(gain));
    }

    pub fn set_structure_gain(&self, gain: f32) {
        store_f32(&self.structure_gain, sanitize_unit(gain));
    }

    fn snapshot(&self) -> EngineControlSnapshot {
        EngineControlSnapshot {
            active: self.active.load(Ordering::Relaxed),
            throttle: sanitize_unit(load_f32(&self.throttle)),
            airborne_gain: sanitize_unit(load_f32(&self.airborne_gain)),
            structure_gain: sanitize_unit(load_f32(&self.structure_gain)),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct EngineControlSnapshot {
    active: bool,
    throttle: f32,
    airborne_gain: f32,
    structure_gain: f32,
}

/// Authored timbral envelope for one broad engine family.
///
/// These values describe synthesis ranges, not authoritative propulsion
/// physics. As richer engine telemetry becomes available, frequency targets
/// can be fed directly instead of inferred from throttle.
#[derive(Debug, Clone, Copy)]
pub struct EngineSynthProfile {
    pub chamber_hz_idle: f32,
    pub chamber_hz_full: f32,
    pub pump_hz_idle: f32,
    pub pump_hz_full: f32,
    pub flow_noise_mix: f32,
    pub chamber_mix: f32,
    pub pump_mix: f32,
}

impl Default for EngineSynthProfile {
    fn default() -> Self {
        Self {
            chamber_hz_idle: 58.0,
            chamber_hz_full: 138.0,
            pump_hz_idle: 220.0,
            pump_hz_full: 640.0,
            flow_noise_mix: 0.58,
            chamber_mix: 0.32,
            pump_mix: 0.10,
        }
    }
}

impl EngineSynthProfile {
    fn sanitized(self) -> Self {
        fn positive(value: f32, fallback: f32) -> f32 {
            if value.is_finite() && value > 0.0 {
                value
            } else {
                fallback
            }
        }

        Self {
            chamber_hz_idle: positive(self.chamber_hz_idle, 58.0),
            chamber_hz_full: positive(self.chamber_hz_full, 138.0),
            pump_hz_idle: positive(self.pump_hz_idle, 220.0),
            pump_hz_full: positive(self.pump_hz_full, 640.0),
            flow_noise_mix: sanitize_unit(self.flow_noise_mix),
            chamber_mix: sanitize_unit(self.chamber_mix),
            pump_mix: sanitize_unit(self.pump_mix),
        }
    }
}

/// Infinite mono PCM generator for one engine.
///
/// The first implementation deliberately has only a few stable components:
/// broadband flow noise, chamber harmonics, and a pump/shaft tone. It is a
/// real procedural source already, but not a final acoustic model.
pub struct EngineSynth {
    control: Arc<EngineSynthControl>,
    profile: EngineSynthProfile,
    sample_rate_hz: f32,
    smoothing: f32,
    smooth_active: f32,
    smooth_throttle: f32,
    smooth_airborne_gain: f32,
    smooth_structure_gain: f32,
    chamber_phase: f32,
    pump_phase: f32,
    rumble_phase: f32,
    noise_state: u32,
    noise_low: f32,
}

impl EngineSynth {
    pub fn new(
        control: Arc<EngineSynthControl>,
        profile: EngineSynthProfile,
        sample_rate_hz: u32,
    ) -> Self {
        let sample_rate_hz = sample_rate_hz.max(8_000) as f32;
        // About 12 ms time constant. This lives in the decoder so control
        // updates never click even when the frame thread changes abruptly.
        let smoothing = 1.0 - (-1.0 / (0.012 * sample_rate_hz)).exp();
        Self {
            control,
            profile: profile.sanitized(),
            sample_rate_hz,
            smoothing,
            smooth_active: 0.0,
            smooth_throttle: 0.0,
            smooth_airborne_gain: 0.0,
            smooth_structure_gain: 0.0,
            chamber_phase: 0.0,
            pump_phase: 0.0,
            rumble_phase: 0.0,
            noise_state: 0x8f31_6d2b,
            noise_low: 0.0,
        }
    }

    pub fn next_sample(&mut self) -> f32 {
        let target = self.control.snapshot();
        let active = if target.active { 1.0 } else { 0.0 };
        self.smooth_active += (active - self.smooth_active) * self.smoothing;
        self.smooth_throttle +=
            (target.throttle - self.smooth_throttle) * self.smoothing;
        self.smooth_airborne_gain +=
            (target.airborne_gain - self.smooth_airborne_gain) * self.smoothing;
        self.smooth_structure_gain +=
            (target.structure_gain - self.smooth_structure_gain) * self.smoothing;

        let throttle = self.smooth_throttle.clamp(0.0, 1.0);
        let chamber_hz = lerp(
            self.profile.chamber_hz_idle,
            self.profile.chamber_hz_full,
            throttle,
        );
        let pump_hz = lerp(
            self.profile.pump_hz_idle,
            self.profile.pump_hz_full,
            throttle,
        );
        let rumble_hz = 17.0 + 12.0 * throttle;

        advance_phase(&mut self.chamber_phase, chamber_hz / self.sample_rate_hz);
        advance_phase(&mut self.pump_phase, pump_hz / self.sample_rate_hz);
        advance_phase(&mut self.rumble_phase, rumble_hz / self.sample_rate_hz);

        let white = self.next_noise();
        // A cheap low-pass state provides correlated low-frequency turbulent
        // energy instead of raw white hiss.
        let noise_alpha = 0.08 + 0.10 * throttle;
        self.noise_low += (white - self.noise_low) * noise_alpha;
        let flow_noise = 0.62 * white + 0.38 * self.noise_low;

        let tau = std::f32::consts::TAU;
        let chamber = (tau * self.chamber_phase).sin()
            + 0.27 * (tau * (self.chamber_phase * 2.03).fract()).sin()
            + 0.11 * (tau * (self.chamber_phase * 3.01).fract()).sin();
        let pump = (tau * self.pump_phase).sin()
            + 0.18 * (tau * (self.pump_phase * 0.5).fract()).sin();
        let rumble = (tau * self.rumble_phase).sin();

        // Airborne sound emphasizes plume/flow noise. Structure-borne sound
        // emphasizes chamber, machinery and low-frequency vibration.
        let airborne = self.smooth_airborne_gain
            * (self.profile.flow_noise_mix * flow_noise
                + self.profile.chamber_mix * 0.55 * chamber
                + self.profile.pump_mix * 0.20 * pump);
        let structure = self.smooth_structure_gain
            * (0.52 * chamber + 0.31 * pump + 0.17 * rumble);

        // At tiny throttle a real engine is not silent, but its acoustic
        // energy is much lower. sqrt gives useful dynamic range without
        // pretending this is a calibrated SPL model.
        let energy = (0.08 + 0.92 * throttle.sqrt()) * self.smooth_active;
        soft_clip((airborne + structure) * energy * 0.52)
    }

    fn next_noise(&mut self) -> f32 {
        let mut x = self.noise_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.noise_state = x;
        let unit = (x as f32) / (u32::MAX as f32);
        unit.mul_add(2.0, -1.0)
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn advance_phase(phase: &mut f32, step: f32) {
    *phase = (*phase + step).fract();
}

fn soft_clip(sample: f32) -> f32 {
    sample / (1.0 + sample.abs())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rms(samples: &[f32]) -> f32 {
        (samples
            .iter()
            .map(|sample| sample * sample)
            .sum::<f32>()
            / samples.len() as f32)
            .sqrt()
    }

    #[test]
    fn default_control_fades_to_silence() {
        let control = Arc::new(EngineSynthControl::default());
        let mut synth =
            EngineSynth::new(control, EngineSynthProfile::default(), DEFAULT_SAMPLE_RATE_HZ);
        let samples: Vec<_> = (0..4096).map(|_| synth.next_sample()).collect();
        assert!(rms(&samples) < 1.0e-6);
    }

    #[test]
    fn live_controls_drive_audio_without_rebuilding_decoder() {
        let control = Arc::new(EngineSynthControl::default());
        let mut synth = EngineSynth::new(
            Arc::clone(&control),
            EngineSynthProfile::default(),
            DEFAULT_SAMPLE_RATE_HZ,
        );
        control.set_active(true);
        control.set_throttle(0.8);
        control.set_airborne_gain(0.8);
        let samples: Vec<_> = (0..8192).map(|_| synth.next_sample()).collect();
        assert!(rms(&samples[4096..]) > 0.02);
        assert!(samples.iter().all(|sample| sample.is_finite()));
        assert!(samples.iter().all(|sample| sample.abs() < 1.0));
    }

    #[test]
    fn airborne_and_structure_routes_are_distinct() {
        let air_control = Arc::new(EngineSynthControl::default());
        air_control.set_active(true);
        air_control.set_throttle(0.7);
        air_control.set_airborne_gain(0.8);

        let structure_control = Arc::new(EngineSynthControl::default());
        structure_control.set_active(true);
        structure_control.set_throttle(0.7);
        structure_control.set_structure_gain(0.8);

        let profile = EngineSynthProfile::default();
        let mut air =
            EngineSynth::new(air_control, profile, DEFAULT_SAMPLE_RATE_HZ);
        let mut structure =
            EngineSynth::new(structure_control, profile, DEFAULT_SAMPLE_RATE_HZ);

        let air_samples: Vec<_> = (0..4096).map(|_| air.next_sample()).collect();
        let structure_samples: Vec<_> =
            (0..4096).map(|_| structure.next_sample()).collect();

        let difference = air_samples
            .iter()
            .zip(&structure_samples)
            .map(|(a, b)| (a - b).abs())
            .sum::<f32>()
            / air_samples.len() as f32;
        assert!(difference > 0.01);
    }
}
