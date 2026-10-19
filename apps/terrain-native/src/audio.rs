//! Native host adapter for the existing acoustic resolver and procedural DSP.
//! Tests render PCM only; the output device is opened exclusively by the window.
use glam::DVec3;
use std::sync::Arc;
use thessa_audio_core::{
    AcousticMedium, AcousticPoint, resolve_airborne_path, structure_path_exists,
};
use thessa_audio_synth::{
    DEFAULT_SAMPLE_RATE_HZ, EngineSynth, EngineSynthControl, EngineSynthProfile,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Options {
    pub muted: bool,
    pub cabin: bool,
    pub demo: bool,
}

/// Observation of an existing flight state, never an authoritative sound event.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub sim_time_s: f64,
    pub engine_active: bool,
    pub throttle: f64,
    pub medium: AcousticMedium,
    pub source: AcousticPoint,
    pub listener: AcousticPoint,
    pub source_cluster: Option<u64>,
    pub listener_cluster: Option<u64>,
    pub rcs_commanded: bool,
    pub gpws_warning: bool,
}

impl Frame {
    /// Isolated boom-fixture clock, with no invented vehicle or physical route.
    pub fn demo_at(time: thessa_sim_core::SimTime) -> Self {
        Self {
            sim_time_s: time.0,
            engine_active: false,
            throttle: 0.0,
            medium: AcousticMedium::VACUUM,
            source: AcousticPoint::stationary(DVec3::ZERO),
            listener: AcousticPoint::stationary(DVec3::ZERO),
            source_cluster: None,
            listener_cluster: None,
            rcs_commanded: false,
            gpws_warning: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Tone {
    frequency_hz: f32,
    duration_ms: u64,
    gain: f32,
}

#[derive(Default)]
struct Mix {
    throttle: f32,
    airborne: f32,
    structure: f32,
    tones: Vec<Tone>,
}

/// Authored listening levels from the disposable client adapter, not calibrated
/// acoustic power. Propagation eligibility still comes from the shared resolver.
struct Policy {
    options: Options,
    rcs: bool,
    gpws: bool,
    last_time_s: Option<f64>,
    boom_arrival_s: Option<f64>,
}

impl Policy {
    fn new(options: Options) -> Self {
        let boom_arrival_s = options
            .demo
            .then(|| {
                thessa_audio_core::sonic_boom_arrival(
                    thessa_audio_core::SupersonicSegment {
                        start_time_s: 0.0,
                        duration_s: 4.0,
                        start_position_m: DVec3::new(-1000.0, 300.0, 0.0),
                        velocity_mps: DVec3::new(500.0, 0.0, 0.0),
                    },
                    DVec3::ZERO,
                    AcousticMedium::gas(1.225, 340.0, 0.0).unwrap(),
                )
                .unwrap()
                .map(|arrival| arrival.arrival_time_s)
            })
            .flatten();
        Self {
            options,
            rcs: false,
            gpws: false,
            last_time_s: None,
            boom_arrival_s,
        }
    }

    fn update(&mut self, frame: Option<Frame>) -> Mix {
        let Some(frame) = frame.filter(|f| f.sim_time_s.is_finite()) else {
            self.rcs = false;
            self.gpws = false;
            self.last_time_s = None;
            return Mix::default();
        };
        let mut mix = Mix::default();
        if !self.options.muted {
            mix.throttle = if frame.throttle.is_finite() {
                frame.throttle.clamp(0.0, 1.0) as f32
            } else {
                0.0
            };
            if frame.engine_active && mix.throttle > 0.0 {
                if self.options.cabin {
                    if structure_path_exists(frame.source_cluster, frame.listener_cluster) {
                        mix.structure = 0.28;
                    }
                } else if resolve_airborne_path(frame.source, frame.listener, frame.medium)
                    .ok()
                    .flatten()
                    .is_some()
                {
                    mix.airborne = 0.34;
                }
            }
            if self.options.cabin {
                if frame.rcs_commanded
                    && !self.rcs
                    && structure_path_exists(frame.source_cluster, frame.listener_cluster)
                {
                    mix.tones.push(Tone {
                        frequency_hz: 260.0,
                        duration_ms: 70,
                        gain: 0.10,
                    });
                }
                if frame.gpws_warning && !self.gpws {
                    // Explicit headset signal, intentionally available in vacuum.
                    mix.tones.push(Tone {
                        frequency_hz: 880.0,
                        duration_ms: 220,
                        gain: 0.18,
                    });
                }
            }
            if let Some(arrival) = self.boom_arrival_s
                && self.last_time_s.is_some_and(|previous| previous < arrival)
                && frame.sim_time_s >= arrival
            {
                mix.tones.extend([
                    Tone {
                        frequency_hz: 55.0,
                        duration_ms: 220,
                        gain: 0.34,
                    },
                    Tone {
                        frequency_hz: 82.0,
                        duration_ms: 140,
                        gain: 0.22,
                    },
                ]);
            }
        }
        self.rcs = frame.rcs_commanded;
        self.gpws = frame.gpws_warning;
        self.last_time_s = Some(frame.sim_time_s);
        mix
    }
}

struct EngineSource(EngineSynth);
impl Iterator for EngineSource {
    type Item = f32;
    fn next(&mut self) -> Option<f32> {
        Some(self.0.next_sample())
    }
}

impl rodio::Source for EngineSource {
    fn current_span_len(&self) -> Option<usize> {
        None
    }
    fn channels(&self) -> rodio::ChannelCount {
        rodio::ChannelCount::new(1).unwrap()
    }
    fn sample_rate(&self) -> rodio::SampleRate {
        rodio::SampleRate::new(DEFAULT_SAMPLE_RATE_HZ).unwrap()
    }
    fn total_duration(&self) -> Option<std::time::Duration> {
        None
    }
}

/// Owns the device lifetime. No graphics/backend/simulation ownership here.
pub struct NativeAudio {
    policy: Policy,
    control: Arc<EngineSynthControl>,
    device: rodio::MixerDeviceSink,
}

impl NativeAudio {
    pub fn open(options: Options) -> Result<Option<Self>, String> {
        if options.muted {
            eprintln!("native_audio=muted; no output device created");
            return Ok(None);
        }
        let mut device = rodio::DeviceSinkBuilder::open_default_sink()
            .map_err(|e| format!("native audio unavailable; continuing silently: {e}"))?;
        device.log_on_drop(false);
        eprintln!(
            "native_audio=active listener={} source=shared_procedural_engine; authored levels, uncalibrated transport",
            if options.cabin {
                "diagnostic_cabin"
            } else {
                "exterior"
            }
        );
        let control = Arc::new(EngineSynthControl::default());
        device.mixer().add(EngineSource(EngineSynth::new(
            control.clone(),
            EngineSynthProfile::default(),
            DEFAULT_SAMPLE_RATE_HZ,
        )));
        Ok(Some(Self {
            policy: Policy::new(options),
            control,
            device,
        }))
    }

    pub fn update(&mut self, frame: Option<Frame>) {
        let mix = self.policy.update(frame);
        self.control.set_throttle(mix.throttle);
        self.control.set_airborne_gain(mix.airborne);
        self.control.set_structure_gain(mix.structure);
        self.control
            .set_active(mix.airborne > 0.0 || mix.structure > 0.0);
        for tone in mix.tones {
            use rodio::Source;
            self.device.mixer().add(
                rodio::source::SineWave::new(tone.frequency_hz)
                    .take_duration(std::time::Duration::from_millis(tone.duration_ms))
                    .amplify(tone.gain),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "benchmark: silent offline target-size 48 kHz audio blocks"]
    fn native_audio_offline_cost() {
        for cabin in [false, true] {
            let mut policy = Policy::new(Options {
                cabin,
                ..Default::default()
            });
            let control = Arc::new(EngineSynthControl::default());
            let mix = policy.update(Some(frame()));
            control.set_throttle(mix.throttle);
            control.set_airborne_gain(mix.airborne);
            control.set_structure_gain(mix.structure);
            control.set_active(true);
            let mut source = EngineSource(EngineSynth::new(
                control,
                EngineSynthProfile::default(),
                DEFAULT_SAMPLE_RATE_HZ,
            ));
            let mut timings = Vec::new();
            for _ in 0..9 {
                let start = std::time::Instant::now();
                for _ in 0..48000 {
                    std::hint::black_box(source.next().unwrap());
                }
                timings.push(start.elapsed().as_secs_f64() * 1000.0);
            }
            timings.sort_by(f64::total_cmp);
            println!(
                "native_audio_cost cabin={cabin} samples=48000 blocks=9 p50_ms={:.6} max_ms={:.6}; offline PCM only, no device or auditory acceptance",
                timings[4], timings[8]
            );
        }
    }
    fn frame() -> Frame {
        Frame {
            sim_time_s: 0.0,
            engine_active: true,
            throttle: 0.8,
            medium: AcousticMedium::gas(1.2, 340.0, 0.0).unwrap(),
            source: AcousticPoint::stationary(DVec3::ZERO),
            listener: AcousticPoint::stationary(DVec3::X * 20.0),
            source_cluster: Some(7),
            listener_cluster: Some(7),
            rcs_commanded: false,
            gpws_warning: false,
        }
    }

    #[test]
    fn exterior_requires_air_and_cabin_requires_connected_structure() {
        let mut policy = Policy::new(Options::default());
        let mut input = frame();
        assert!(policy.update(Some(input)).airborne > 0.0);
        input.medium = AcousticMedium::VACUUM;
        assert_eq!(policy.update(Some(input)).airborne, 0.0);
        let mut policy = Policy::new(Options {
            cabin: true,
            ..Default::default()
        });
        assert!(policy.update(Some(input)).structure > 0.0);
        input.listener_cluster = Some(8);
        assert_eq!(policy.update(Some(input)).structure, 0.0);
        input.listener_cluster = None;
        assert_eq!(policy.update(Some(input)).structure, 0.0);
    }

    #[test]
    fn warning_and_rcs_are_edges_not_per_frame_tones() {
        let mut policy = Policy::new(Options {
            cabin: true,
            ..Default::default()
        });
        let mut input = frame();
        input.medium = AcousticMedium::VACUUM;
        input.gpws_warning = true;
        input.rcs_commanded = true;
        assert_eq!(policy.update(Some(input)).tones.len(), 2);
        assert!(policy.update(Some(input)).tones.is_empty());
        policy.update(None);
        assert_eq!(policy.update(Some(input)).tones.len(), 2);
        input.listener_cluster = None;
        policy.update(None);
        let tones = policy.update(Some(input)).tones;
        assert_eq!(tones.len(), 1);
        assert_eq!(tones[0].frequency_hz, 880.0);
        let mut exterior = Policy::new(Options::default());
        assert!(exterior.update(Some(input)).tones.is_empty());
    }

    #[test]
    fn mute_and_invalid_state_are_silent_without_opening_any_device() {
        assert!(
            NativeAudio::open(Options {
                muted: true,
                ..Default::default()
            })
            .unwrap()
            .is_none()
        );
        let mut policy = Policy::new(Options {
            muted: true,
            cabin: true,
            demo: true,
        });
        let mut input = frame();
        input.gpws_warning = true;
        input.rcs_commanded = true;
        let mix = policy.update(Some(input));
        assert_eq!(mix.airborne + mix.structure, 0.0);
        assert!(mix.tones.is_empty());
        input.sim_time_s = f64::NAN;
        let mix = policy.update(Some(input));
        assert_eq!(mix.airborne + mix.structure, 0.0);
        assert!(mix.tones.is_empty());
        assert_eq!(policy.update(None).airborne, 0.0);
    }

    #[test]
    fn boom_demo_follows_sim_time_pause_reverse_and_crossing() {
        let mut policy = Policy::new(Options {
            demo: true,
            ..Default::default()
        });
        let arrival = policy.boom_arrival_s.unwrap();
        let mut input = frame();
        input.sim_time_s = arrival - 0.1;
        assert!(policy.update(Some(input)).tones.is_empty());
        assert!(policy.update(Some(input)).tones.is_empty());
        input.sim_time_s = arrival + 0.1;
        assert_eq!(policy.update(Some(input)).tones.len(), 2);
        assert!(policy.update(Some(input)).tones.is_empty());
        input.sim_time_s = arrival - 1.0;
        assert!(policy.update(Some(input)).tones.is_empty());
        input.sim_time_s = arrival + 1.0;
        assert_eq!(policy.update(Some(input)).tones.len(), 2);
    }

    #[test]
    fn engine_decoder_produces_bounded_pcm_offline_and_mutes_in_vacuum() {
        let control = Arc::new(EngineSynthControl::default());
        let mut source = EngineSource(EngineSynth::new(
            control.clone(),
            EngineSynthProfile::default(),
            DEFAULT_SAMPLE_RATE_HZ,
        ));
        let mut policy = Policy::new(Options::default());
        let mix = policy.update(Some(frame()));
        control.set_throttle(mix.throttle);
        control.set_airborne_gain(mix.airborne);
        control.set_active(true);
        let pcm: Vec<_> = source.by_ref().take(48000).collect();
        assert!(pcm.iter().all(|v| v.is_finite() && v.abs() <= 1.0));
        assert!(pcm.iter().any(|v| v.abs() > 0.001));
        let mut input = frame();
        input.medium = AcousticMedium::VACUUM;
        let mix = policy.update(Some(input));
        control.set_airborne_gain(mix.airborne);
        control.set_active(false);
        let tail: Vec<_> = source.by_ref().take(48000).collect();
        assert!(tail[24000..].iter().all(|v| v.abs() < 1e-8));
    }
}
