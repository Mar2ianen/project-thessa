//! Authoritative flight wire messages (game layer, GPL).
//!
//! The server owns the flight state; clients send [`ClientInput`] and apply
//! [`Snapshot`]. Vectors are plain `[f64; 3]` arrays so the wire format does
//! not couple to any math crate version. Framing and versioning come from
//! `thessa-protocol`; this crate only owns the game payload registry.

use serde::{Deserialize, Serialize};
use thessa_autopilot::{AutopilotGraph, PlanDeoptimizationReason, TrajectoryPlan};
use thessa_flight_authority::ControlMode;
use thessa_flight_control::{GuidanceIntent, PropulsionDemand};
use thessa_protocol::{CodecError, Envelope, kind};
use thessa_sim_core::RigidBodyState;

/// Client handshake: version check happens on [`Envelope`], this carries
/// the human-readable identity for logs and the admin surface.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Hello {
    pub client_name: String,
}

/// Server handshake reply: the tick and sim time the client must adopt.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Welcome {
    pub tick: u64,
    pub flight_time_s: f64,
}

/// Discrete commands bundled with an input (warp votes, staging, toggles).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Command {
    SetWarp {
        factor: f64,
    },
    Stage,
    Engine {
        active: bool,
    },
    Pause {
        paused: bool,
    },
    /// Relaunch at the canonical survey site, preserving the clock. The
    /// server derives the site from the same baked-in recipe as the client
    /// survey, so this event carries no world state — only player intent,
    /// like staging. Forces a prompt snapshot like staging does.
    Reset,
}

/// Per-tick pilot input. The server applies the latest input per client;
/// there is deliberately no catch-up queue (AGENTS.md multiplayer rule:
/// input/commands, never world state).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ClientInput {
    /// Client's view of the tick this input targets; the server clamps.
    pub tick: u64,
    /// Manual body-axis command: pitch, yaw, roll in normalized units.
    pub control_input: [f64; 3],
    /// Assist mode selecting the server-side control law.
    pub control_mode: ControlMode,
    /// SAS attitude target as (x, y, z, w); ignored unless finite nonzero.
    pub sas_target_xyzw: [f64; 4],
    pub throttle: f64,
    pub engine_active: bool,
    pub sas_enabled: bool,
    pub rcs_enabled: bool,
    pub gear_down: bool,
    pub commands: Vec<Command>,
}

/// Typed guidance command introduced beside [`ClientInput`] so peers can
/// migrate without making the legacy SAS representation permanent. The
/// server still validates and realizes it through native control laws.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GuidanceInput {
    pub tick: u64,
    pub intent: GuidanceIntent,
    pub propulsion: PropulsionDemand,
}

impl GuidanceInput {
    pub fn validate(&self) -> Result<(), thessa_flight_control::ControlError> {
        self.intent.validate()?;
        PropulsionDemand::new(self.propulsion.normalized).map(|_| ())
    }
}

/// Authoritative per-tick snapshot for one vehicle. Lean by design: render
///-only derivations (regime labels, map projections) stay client-side.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Snapshot {
    pub tick: u64,
    pub flight_time_s: f64,
    pub state: RigidBodyState,
    pub throttle: f64,
    pub engine_active: bool,
    pub paused: bool,
    pub effective_warp: f64,
    /// Cumulative authoritative-server wall durations. These are separate
    /// from the client render/simulation CPU timings and make the reported
    /// warp reproducible as advanced sim time per real elapsed second.
    pub server_compute_s: f64,
    pub server_wall_s: f64,
    pub steps_this_frame: u32,
    pub rails_advanced_s: f64,
    pub wake_notice: Option<String>,
    pub flight_error: Option<String>,
}

/// Encode a game message into a framed byte stream chunk.
pub fn encode_frame(kind: u32, message: &impl Serialize) -> Result<Vec<u8>, CodecError> {
    let envelope = thessa_protocol::encode_envelope(kind, message)?;
    thessa_protocol::encode_frame(&envelope)
}

/// Decode one complete frame into its envelope (version-checked).
pub fn decode_frame(bytes: &[u8]) -> Result<Envelope, CodecError> {
    thessa_protocol::decode_envelope(bytes)
}

/// Deserialize an envelope payload by kind.
pub fn decode_payload<T: for<'a> Deserialize<'a>>(envelope: &Envelope) -> Result<T, CodecError> {
    postcard::from_bytes(&envelope.payload).map_err(|e| CodecError::Codec(e.to_string()))
}

pub fn encode_input(input: &ClientInput) -> Result<Vec<u8>, CodecError> {
    encode_frame(kind::CLIENT_INPUT, input)
}

pub fn encode_guidance(input: &GuidanceInput) -> Result<Vec<u8>, CodecError> {
    encode_frame(kind::GUIDANCE_COMMAND, input)
}

/// High-level automation operations. The authoritative server owns execution
/// and validates every plan/script before it can affect a vehicle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AutopilotCommand {
    SubmitGraph { graph: AutopilotGraph },
    ClearGraph,
    StartScript { source: String },
    SubmitPlan { plan: TrajectoryPlan },
    Cancel,
    Deoptimize { reason: PlanDeoptimizationReason },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AutopilotInput {
    pub tick: u64,
    pub command: AutopilotCommand,
}

impl AutopilotInput {
    pub fn validate(&self) -> Result<(), String> {
        match &self.command {
            AutopilotCommand::SubmitGraph { graph } => {
                graph.validate().map(|_| ()).map_err(|errors| {
                    errors
                        .into_iter()
                        .map(|error| error.to_string())
                        .collect::<Vec<_>>()
                        .join("; ")
                })
            }
            AutopilotCommand::StartScript { source } if source.trim().is_empty() => {
                Err("autopilot script must not be empty".into())
            }
            AutopilotCommand::SubmitPlan { plan } => {
                plan.validate().map_err(|error| error.to_string())
            }
            _ => Ok(()),
        }
    }
}

pub fn encode_snapshot(snapshot: &Snapshot) -> Result<Vec<u8>, CodecError> {
    encode_frame(kind::SNAPSHOT, snapshot)
}

pub fn encode_autopilot(input: &AutopilotInput) -> Result<Vec<u8>, CodecError> {
    encode_frame(kind::AUTOPILOT_COMMAND, input)
}

pub fn encode_hello(hello: &Hello) -> Result<Vec<u8>, CodecError> {
    encode_frame(kind::HELLO, hello)
}

pub fn encode_welcome(welcome: &Welcome) -> Result<Vec<u8>, CodecError> {
    encode_frame(kind::WELCOME, welcome)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};
    use thessa_protocol::FrameDecoder;

    fn sample_input() -> ClientInput {
        ClientInput {
            tick: 7200,
            control_input: [0.1, -0.2, 0.0],
            control_mode: ControlMode::Navball,
            sas_target_xyzw: [0.0, 0.0, 0.0, 1.0],
            throttle: 0.65,
            engine_active: true,
            sas_enabled: true,
            rcs_enabled: false,
            gear_down: false,
            commands: vec![Command::SetWarp { factor: 128.0 }, Command::Stage],
        }
    }

    fn sample_snapshot() -> Snapshot {
        Snapshot {
            tick: 7200,
            flight_time_s: 60.0,
            state: RigidBodyState::new(
                DVec3::new(6.5e6, 0.0, 0.0),
                DVec3::new(0.0, 7800.0, 0.0),
                DQuat::IDENTITY,
                DVec3::ZERO,
            )
            .expect("valid state"),
            throttle: 0.65,
            engine_active: true,
            paused: false,
            effective_warp: 35.2,
            server_compute_s: 0.042,
            server_wall_s: 1.705,
            steps_this_frame: 181,
            rails_advanced_s: 0.0,
            wake_notice: None,
            flight_error: None,
        }
    }

    #[test]
    fn typed_autopilot_plan_roundtrips() {
        let input = AutopilotInput {
            tick: 41,
            command: AutopilotCommand::SubmitPlan {
                plan: TrajectoryPlan {
                    id: thessa_flight_control::TrajectoryPlanId(12),
                    segments: vec![thessa_autopilot::TrajectorySegment::Coast { duration_s: 4.0 }],
                    bakeability: thessa_autopilot::Bakeability::Pure,
                },
            },
        };
        input.validate().expect("valid plan");
        let frame = encode_autopilot(&input).expect("frame");
        let mut decoder = FrameDecoder::new();
        let frames = decoder.push(&frame).expect("framed envelope");
        let envelope = decode_frame(&frames[0]).expect("envelope");
        assert_eq!(envelope.kind, kind::AUTOPILOT_COMMAND);
        let back: AutopilotInput = decode_payload(&envelope).expect("payload");
        assert_eq!(back, input);
    }

    #[test]
    fn typed_autopilot_graph_validates_and_roundtrips() {
        let input = AutopilotInput {
            tick: 42,
            command: AutopilotCommand::SubmitGraph {
                graph: thessa_autopilot::AutopilotGraph {
                    nodes: vec![
                        thessa_autopilot::GraphNode {
                            id: thessa_autopilot::NodeId(1),
                            name: "source".into(),
                            kind: thessa_autopilot::NodeKind::Source,
                            ports: vec![thessa_autopilot::Port::output(
                                "value",
                                thessa_autopilot::PortType::Number,
                            )],
                            config: None,
                        },
                        thessa_autopilot::GraphNode {
                            id: thessa_autopilot::NodeId(2),
                            name: "sink".into(),
                            kind: thessa_autopilot::NodeKind::Sink,
                            ports: vec![thessa_autopilot::Port::input(
                                "value",
                                thessa_autopilot::PortType::Number,
                                true,
                            )],
                            config: None,
                        },
                    ],
                    edges: vec![thessa_autopilot::GraphEdge {
                        from: thessa_autopilot::PortRef {
                            node: thessa_autopilot::NodeId(1),
                            port: "value".into(),
                        },
                        to: thessa_autopilot::PortRef {
                            node: thessa_autopilot::NodeId(2),
                            port: "value".into(),
                        },
                    }],
                },
            },
        };
        input.validate().expect("valid graph");
        let frame = encode_autopilot(&input).expect("frame");
        let mut decoder = FrameDecoder::new();
        let frames = decoder.push(&frame).expect("framed envelope");
        let envelope = decode_frame(&frames[0]).expect("envelope");
        let back: AutopilotInput = decode_payload(&envelope).expect("payload");
        assert_eq!(back, input);
    }

    #[test]
    fn typed_guidance_input_roundtrips_alongside_legacy_input() {
        let input = GuidanceInput {
            tick: 12,
            intent: GuidanceIntent::ManualAxes(Default::default()),
            propulsion: PropulsionDemand::new(1.0).unwrap(),
        };
        let frame = encode_guidance(&input).expect("encode");
        let mut decoder = FrameDecoder::new();
        let frames = decoder.push(&frame).expect("split");
        let envelope = decode_frame(&frames[0]).expect("envelope");
        assert_eq!(envelope.kind, kind::GUIDANCE_COMMAND);
        let back: GuidanceInput = decode_payload(&envelope).expect("payload");
        assert_eq!(back, input);
        back.validate().expect("valid guidance");
    }

    #[test]
    fn input_roundtrips_through_framed_stream() {
        let input = sample_input();
        let frame = encode_input(&input).expect("encode");
        let mut decoder = FrameDecoder::new();
        let frames = decoder.push(&frame).expect("split");
        assert_eq!(frames.len(), 1);
        let envelope = decode_frame(&frames[0]).expect("envelope");
        assert_eq!(envelope.kind, kind::CLIENT_INPUT);
        let back: ClientInput = decode_payload(&envelope).expect("payload");
        assert_eq!(back, input);
    }

    #[test]
    fn snapshot_roundtrips_with_full_state() {
        let snapshot = sample_snapshot();
        let frame = encode_snapshot(&snapshot).expect("encode");
        let mut decoder = FrameDecoder::new();
        let frames = decoder.push(&frame).expect("split");
        assert_eq!(frames.len(), 1);
        let envelope = decode_frame(&frames[0]).expect("envelope");
        assert_eq!(envelope.kind, kind::SNAPSHOT);
        let back: Snapshot = decode_payload(&envelope).expect("payload");
        assert_eq!(back, snapshot);
    }

    #[test]
    fn hello_welcome_handshake_roundtrips() {
        let hello = Hello {
            client_name: "local-stdio".into(),
        };
        let welcome = Welcome {
            tick: 0,
            flight_time_s: 0.0,
        };
        let mut decoder = FrameDecoder::new();
        let mut stream = encode_hello(&hello).expect("hello");
        stream.extend_from_slice(&encode_welcome(&welcome).expect("welcome"));
        // Split mid-frame: handshake must survive pipe chunking.
        let at = stream.len() / 2;
        let mut frames = decoder.push(&stream[..at]).expect("push");
        frames.extend(decoder.push(&stream[at..]).expect("push"));
        assert_eq!(frames.len(), 2);
        let first = decode_frame(&frames[0]).expect("env");
        let second = decode_frame(&frames[1]).expect("env");
        assert_eq!(first.kind, kind::HELLO);
        assert_eq!(second.kind, kind::WELCOME);
        let back_hello: Hello = decode_payload(&first).expect("hello payload");
        let back_welcome: Welcome = decode_payload(&second).expect("welcome payload");
        assert_eq!(back_hello, hello);
        assert_eq!(back_welcome, welcome);
    }

    #[test]
    fn input_snapshot_loopback_over_simulated_pipe() {
        // Client -> server -> client through two decoders and a Vec<u8>
        // standing in for the stdio pipe / TCP stream.
        let input = sample_input();
        let mut client_to_server = FrameDecoder::new();
        let mut server_to_client = FrameDecoder::new();
        let wire = encode_input(&input).expect("client encodes");
        let at_server = client_to_server.push(&wire).expect("server splits");
        assert_eq!(at_server.len(), 1);
        let server_input: ClientInput =
            decode_payload(&decode_frame(&at_server[0]).expect("env")).expect("input");
        assert_eq!(server_input.commands.len(), 2);
        // Server answers with a snapshot derived from the input tick.
        let mut snapshot = sample_snapshot();
        snapshot.tick = server_input.tick;
        let reply = encode_snapshot(&snapshot).expect("server encodes");
        let at_client = server_to_client.push(&reply).expect("client splits");
        assert_eq!(at_client.len(), 1);
        let back: Snapshot =
            decode_payload(&decode_frame(&at_client[0]).expect("env")).expect("snapshot");
        assert_eq!(back.tick, input.tick);
        assert_eq!(back, snapshot);
    }
}

#[cfg(test)]
mod reset_command_tests {
    use super::*;
    use thessa_protocol::FrameDecoder;

    #[test]
    fn reset_command_roundtrips_and_preserves_order() {
        let input = ClientInput {
            tick: 99,
            control_input: [0.0; 3],
            control_mode: ControlMode::Direct,
            sas_target_xyzw: [0.0, 0.0, 0.0, 1.0],
            throttle: 0.0,
            engine_active: false,
            sas_enabled: false,
            rcs_enabled: false,
            gear_down: false,
            commands: vec![
                Command::Stage,
                Command::Reset,
                Command::SetWarp { factor: 8.0 },
            ],
        };
        let frame = encode_input(&input).expect("encode");
        let mut decoder = FrameDecoder::new();
        let frames = decoder.push(&frame).expect("split");
        let back: ClientInput =
            decode_payload(&decode_frame(&frames[0]).expect("env")).expect("payload");
        assert_eq!(back, input);
        assert_eq!(back.commands[1], Command::Reset);
    }
}
