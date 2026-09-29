//! Temporary Bevy adapter for Thessa's backend-neutral audio semantics.
//!
//! Physical routing lives in `thessa-audio-core`; procedural DSP lives in
//! `thessa-audio-synth`. This module only adapts those layers to Bevy's
//! current audio backend and can disappear with the Bevy host.

use std::{sync::Arc, time::Duration};

use bevy::{
    audio::{AddAudioSource, ChannelCount, Decodable, SampleRate, Source, Volume},
    prelude::*,
    reflect::TypePath,
};
use glam::DVec3;
use thessa_audio_core::{
    AcousticMedium, AcousticPoint, SupersonicSegment, resolve_airborne_path, sonic_boom_arrival,
    structure_path_exists,
};
use thessa_audio_synth::{
    DEFAULT_SAMPLE_RATE_HZ, EngineSynth, EngineSynthControl, EngineSynthProfile,
};

use crate::pilot::{ClientViewMode, PilotFlightRuntime, PilotHudState};

const ENGINE_AIRBORNE_GAIN: f32 = 0.34;
const ENGINE_STRUCTURE_GAIN: f32 = 0.28;
const GPWS_HZ: f32 = 880.0;
const RCS_HZ: f32 = 260.0;
const DOCK_HZ: f32 = 110.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ListenerMode {
    Exterior,
    Cabin,
}

#[derive(Resource)]
struct AudioRuntime {
    listener_mode: ListenerMode,
    engine_control: Arc<EngineSynthControl>,
    previous_rcs_commanded: bool,
    previous_gpws_warning: bool,
    demo_boom_remaining_s: Option<f64>,
}

impl Default for AudioRuntime {
    fn default() -> Self {
        Self {
            listener_mode: if std::env::args().any(|arg| arg == "--audio-cabin") {
                ListenerMode::Cabin
            } else {
                ListenerMode::Exterior
            },
            engine_control: Arc::new(EngineSynthControl::default()),
            previous_rcs_commanded: false,
            previous_gpws_warning: false,
            demo_boom_remaining_s: None,
        }
    }
}

/// Bevy-only wrapper around the backend-neutral procedural engine DSP.
#[derive(Asset, TypePath)]
struct ProceduralEngineAudio {
    control: Arc<EngineSynthControl>,
    profile: EngineSynthProfile,
}

struct ProceduralEngineDecoder {
    synth: EngineSynth,
    sample_rate: SampleRate,
    channels: ChannelCount,
}

impl Iterator for ProceduralEngineDecoder {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        Some(self.synth.next_sample())
    }
}

impl Source for ProceduralEngineDecoder {
    fn current_span_len(&self) -> Option<usize> {
        None
    }

    fn channels(&self) -> ChannelCount {
        self.channels
    }

    fn sample_rate(&self) -> SampleRate {
        self.sample_rate
    }

    fn total_duration(&self) -> Option<Duration> {
        None
    }
}

impl Decodable for ProceduralEngineAudio {
    type Decoder = ProceduralEngineDecoder;

    fn decoder(&self) -> Self::Decoder {
        ProceduralEngineDecoder {
            synth: EngineSynth::new(
                Arc::clone(&self.control),
                self.profile,
                DEFAULT_SAMPLE_RATE_HZ,
            ),
            sample_rate: SampleRate::new(DEFAULT_SAMPLE_RATE_HZ)
                .expect("48 kHz procedural audio sample rate must be valid"),
            channels: ChannelCount::new(1).expect("mono procedural audio must be valid"),
        }
    }
}

/// Semantic one-shot bridge used by client fixtures until their physical
/// events are published by a shared gameplay/event layer.
#[derive(Message, Debug, Clone, Copy)]
pub(super) enum AudioCue {
    DockingImpact { relative_speed_mps: f64 },
}

pub(super) struct ThessaAudioPlugin;

impl Plugin for ThessaAudioPlugin {
    fn build(&self, app: &mut App) {
        app.add_audio_source::<ProceduralEngineAudio>()
            .init_resource::<AudioRuntime>()
            .add_message::<AudioCue>()
            .add_systems(Startup, setup_audio)
            .add_systems(
                Update,
                (
                    update_engine_voice,
                    emit_rcs_impulse,
                    emit_gpws_warning,
                    handle_audio_cues,
                    tick_demo_boom,
                ),
            );
    }
}

fn setup_audio(
    mut commands: Commands,
    mut engines: ResMut<Assets<ProceduralEngineAudio>>,
    mut runtime: ResMut<AudioRuntime>,
) {
    let engine = engines.add(ProceduralEngineAudio {
        control: Arc::clone(&runtime.engine_control),
        profile: EngineSynthProfile::default(),
    });
    commands.spawn((
        AudioPlayer(engine),
        PlaybackSettings::LOOP.with_volume(Volume::Linear(1.0)),
        Name::new("Thessa procedural engine voice"),
    ));

    if std::env::args().any(|arg| arg == "--audio-demo") {
        let medium =
            AcousticMedium::gas(1.225, 340.0, 0.0).expect("audio demo atmosphere must be valid");
        let segment = SupersonicSegment {
            start_time_s: 0.0,
            duration_s: 4.0,
            start_position_m: DVec3::new(-1_000.0, 300.0, 0.0),
            velocity_mps: DVec3::new(500.0, 0.0, 0.0),
        };
        if let Some(arrival) = sonic_boom_arrival(segment, DVec3::ZERO, medium)
            .expect("audio demo boom geometry must be valid")
        {
            runtime.demo_boom_remaining_s = Some(arrival.arrival_time_s);
            eprintln!(
                "[audio] sonic-boom demo armed: source passes x=0 at 2.000s, boom arrives at {:.3}s",
                arrival.arrival_time_s
            );
        }
    }

    eprintln!(
        "[audio] listener={:?}; --audio-cabin selects the temporary cabin test listener",
        runtime.listener_mode
    );
}

fn update_engine_voice(
    runtime: Res<PilotFlightRuntime>,
    hud: Res<PilotHudState>,
    audio: Res<AudioRuntime>,
) {
    let control = &audio.engine_control;
    let engine_running =
        hud.view_mode == ClientViewMode::Pilot && runtime.engine_active && runtime.throttle > 0.0;

    control.set_throttle(runtime.throttle.clamp(0.0, 1.0) as f32);

    if !engine_running {
        control.set_active(false);
        control.set_airborne_gain(0.0);
        control.set_structure_gain(0.0);
        return;
    }

    let altitude_m = (runtime.relative_position_m.length() - runtime.planet_radius_m).max(0.0);
    let sample = runtime.atmosphere.sample(altitude_m).ok();
    let medium = match sample {
        Some(sample) if sample.density_kg_m3 > 0.0 => {
            AcousticMedium::gas(sample.density_kg_m3, sample.speed_of_sound_mps, 0.0)
                .unwrap_or(AcousticMedium::VACUUM)
        }
        _ => AcousticMedium::VACUUM,
    };

    // The pilot camera is currently an exterior chase camera. Cabin mode is a
    // debug listener until true IVA exists; it exercises the same structural
    // route the future crew listener will use.
    let (airborne_gain, structure_gain) = match audio.listener_mode {
        ListenerMode::Exterior => {
            let source = AcousticPoint::stationary(DVec3::ZERO);
            let listener =
                AcousticPoint::stationary(DVec3::new(hud.audio_camera_distance_m(), 0.0, 0.0));
            let airborne = resolve_airborne_path(source, listener, medium)
                .ok()
                .flatten()
                .is_some();
            (if airborne { ENGINE_AIRBORNE_GAIN } else { 0.0 }, 0.0)
        }
        ListenerMode::Cabin => (
            0.0,
            if structure_path_exists(Some(1), Some(1)) {
                ENGINE_STRUCTURE_GAIN
            } else {
                0.0
            },
        ),
    };

    control.set_airborne_gain(airborne_gain);
    control.set_structure_gain(structure_gain);
    control.set_active(airborne_gain > 0.0 || structure_gain > 0.0);
}

fn emit_rcs_impulse(
    runtime: Res<PilotFlightRuntime>,
    hud: Res<PilotHudState>,
    mut audio: ResMut<AudioRuntime>,
    mut pitches: ResMut<Assets<Pitch>>,
    mut commands: Commands,
) {
    let commanded = hud.view_mode == ClientViewMode::Pilot
        && runtime.rcs_enabled
        && runtime.regime() == crate::pilot::FlightRegime::Coast
        && runtime.control_input.length_squared() > 0.02;

    if commanded && !audio.previous_rcs_commanded && audio.listener_mode == ListenerMode::Cabin {
        spawn_tone(&mut commands, &mut pitches, RCS_HZ, 70, 0.10);
    }
    audio.previous_rcs_commanded = commanded;
}

fn emit_gpws_warning(
    hud: Res<PilotHudState>,
    mut audio: ResMut<AudioRuntime>,
    mut pitches: ResMut<Assets<Pitch>>,
    mut commands: Commands,
) {
    let warning = hud.view_mode == ClientViewMode::Pilot
        && hud.audio_warning_active("LOW ALTITUDE / DESCENDING");

    if warning && !audio.previous_gpws_warning && audio.listener_mode == ListenerMode::Cabin {
        // Direct cockpit signal: intentionally independent of atmosphere.
        spawn_tone(&mut commands, &mut pitches, GPWS_HZ, 220, 0.18);
    }
    audio.previous_gpws_warning = warning;
}

fn handle_audio_cues(
    mut cues: MessageReader<AudioCue>,
    audio: Res<AudioRuntime>,
    mut pitches: ResMut<Assets<Pitch>>,
    mut commands: Commands,
) {
    for cue in cues.read() {
        match *cue {
            AudioCue::DockingImpact { relative_speed_mps } => {
                // The docking demo is a vacuum fixture. Exterior physical
                // audio is therefore silent; a cabin listener hears the
                // structure-borne capture impulse.
                if audio.listener_mode == ListenerMode::Cabin {
                    let gain = (relative_speed_mps / 0.05).clamp(0.05, 1.0) as f32 * 0.24;
                    spawn_tone(&mut commands, &mut pitches, DOCK_HZ, 180, gain);
                }
            }
        }
    }
}

fn tick_demo_boom(
    time: Res<Time>,
    mut audio: ResMut<AudioRuntime>,
    mut pitches: ResMut<Assets<Pitch>>,
    mut commands: Commands,
) {
    let Some(remaining) = audio.demo_boom_remaining_s else {
        return;
    };
    let next = remaining - time.delta_secs_f64();
    if next > 0.0 {
        audio.demo_boom_remaining_s = Some(next);
        return;
    }

    audio.demo_boom_remaining_s = None;
    // Two short low pitches make the propagation test obvious without adding
    // a committed sound asset. This is presentation only.
    spawn_tone(&mut commands, &mut pitches, 55.0, 220, 0.34);
    spawn_tone(&mut commands, &mut pitches, 82.0, 140, 0.22);
    eprintln!("[audio] delayed sonic-boom demo arrived");
}

fn spawn_tone(
    commands: &mut Commands,
    pitches: &mut Assets<Pitch>,
    frequency_hz: f32,
    duration_ms: u64,
    volume: f32,
) {
    let pitch = pitches.add(Pitch::new(frequency_hz, Duration::from_millis(duration_ms)));
    commands.spawn((
        AudioPlayer(pitch),
        PlaybackSettings::DESPAWN.with_volume(Volume::Linear(volume.clamp(0.0, 1.0))),
    ));
}
