//! Reusable wire/domain protocol primitives.
//!
//! Transport-agnostic by construction: [`Envelope`] is a versioned,
//! kind-tagged postcard wrapper and [`FrameDecoder`] turns an arbitrary
//! byte stream (stdio pipe, TCP socket, in-memory queue) into length
//! prefixed frames. Async adapters live in the apps; this crate never
//! touches Tokio, Bevy or game types (licensing boundary: MIT).

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolVersion(pub u16);

impl ProtocolVersion {
    pub const CURRENT: Self = Self(1);
}

/// Numeric message kind. Game payloads assign their own registry in the
/// shared flight crate; the envelope layer only routes and rejects.
pub mod kind {
    pub const HELLO: u32 = 1;
    pub const WELCOME: u32 = 2;
    pub const CLIENT_INPUT: u32 = 3;
    pub const SNAPSHOT: u32 = 4;
    pub const COMMAND: u32 = 5;
    pub const EVENT: u32 = 6;
}

/// Versioned envelope around one postcard-encoded game payload.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Envelope {
    pub version: ProtocolVersion,
    pub kind: u32,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CodecError {
    /// Length prefix exceeds [`MAX_FRAME_BYTES`] (corrupt or hostile peer).
    FrameTooLarge { declared: usize },
    /// postcard (de)serialization failure with the crate's error string.
    Codec(String),
    /// Peer speaks a version we do not understand.
    VersionMismatch { got: u16 },
}

impl std::fmt::Display for CodecError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::FrameTooLarge { declared } => {
                write!(formatter, "frame declares {declared} bytes, over the limit")
            }
            Self::Codec(message) => write!(formatter, "codec error: {message}"),
            Self::VersionMismatch { got } => {
                write!(formatter, "protocol version mismatch: peer sent {got}")
            }
        }
    }
}

impl std::error::Error for CodecError {}

/// Hard cap on a single frame payload (envelope included). Snapshots are
/// small state vectors, never bulk assets; bulk data gets its own channel.
pub const MAX_FRAME_BYTES: usize = 8 * 1024 * 1024;

/// Encode a game message into a versioned envelope.
pub fn encode_envelope<T: Serialize>(kind: u32, message: &T) -> Result<Vec<u8>, CodecError> {
    let payload = postcard::to_allocvec(message).map_err(|e| CodecError::Codec(e.to_string()))?;
    let envelope = Envelope {
        version: ProtocolVersion::CURRENT,
        kind,
        payload,
    };
    postcard::to_allocvec(&envelope).map_err(|e| CodecError::Codec(e.to_string()))
}

/// Decode and version-check an envelope; payload stays opaque bytes for the
/// game layer to deserialize by `kind`.
pub fn decode_envelope(bytes: &[u8]) -> Result<Envelope, CodecError> {
    let envelope: Envelope =
        postcard::from_bytes(bytes).map_err(|e| CodecError::Codec(e.to_string()))?;
    if envelope.version != ProtocolVersion::CURRENT {
        return Err(CodecError::VersionMismatch {
            got: envelope.version.0,
        });
    }
    Ok(envelope)
}

/// Length-prefix one envelope buffer for the byte stream (u32 LE, no padding).
pub fn encode_frame(envelope_bytes: &[u8]) -> Result<Vec<u8>, CodecError> {
    if envelope_bytes.len() > MAX_FRAME_BYTES {
        return Err(CodecError::FrameTooLarge {
            declared: envelope_bytes.len(),
        });
    }
    let mut frame = Vec::with_capacity(4 + envelope_bytes.len());
    frame.extend_from_slice(&(envelope_bytes.len() as u32).to_le_bytes());
    frame.extend_from_slice(envelope_bytes);
    Ok(frame)
}

/// Incremental frame splitter: feed whatever the transport yields
/// (a partial pipe read, a TCP segment), pop complete envelopes.
#[derive(Debug, Default)]
pub struct FrameDecoder {
    buffer: Vec<u8>,
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Push newly arrived bytes; returns every envelope completed by them.
    /// A corrupt length prefix is a hard error: the stream cannot resync.
    pub fn push(&mut self, bytes: &[u8]) -> Result<Vec<Vec<u8>>, CodecError> {
        self.buffer.extend_from_slice(bytes);
        let mut frames = Vec::new();
        loop {
            if self.buffer.len() < 4 {
                break;
            }
            let declared = u32::from_le_bytes([
                self.buffer[0],
                self.buffer[1],
                self.buffer[2],
                self.buffer[3],
            ]) as usize;
            if declared > MAX_FRAME_BYTES {
                return Err(CodecError::FrameTooLarge { declared });
            }
            if self.buffer.len() < 4 + declared {
                break;
            }
            frames.push(self.buffer[4..4 + declared].to_vec());
            self.buffer.drain(..4 + declared);
        }
        Ok(frames)
    }

    pub fn pending_bytes(&self) -> usize {
        self.buffer.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    struct TestInput {
        tick: u64,
        throttle: u32,
        warp: u32,
    }

    #[test]
    fn envelope_roundtrip_preserves_kind_and_payload() {
        let message = TestInput {
            tick: 7200,
            throttle: 65,
            warp: 128,
        };
        let bytes = encode_envelope(kind::CLIENT_INPUT, &message).expect("encode");
        let envelope = decode_envelope(&bytes).expect("decode");
        assert_eq!(envelope.kind, kind::CLIENT_INPUT);
        let back: TestInput = postcard::from_bytes(&envelope.payload).expect("payload");
        assert_eq!(back, message);
    }

    #[test]
    fn frames_survive_arbitrary_splitting() {
        let first = encode_envelope(
            kind::CLIENT_INPUT,
            &TestInput {
                tick: 1,
                throttle: 0,
                warp: 1,
            },
        )
        .expect("encode");
        let second = encode_envelope(kind::COMMAND, &42u32).expect("encode");
        let mut stream = encode_frame(&first).expect("frame");
        stream.extend_from_slice(&encode_frame(&second).expect("frame"));
        // Byte-at-a-time delivery, worst case for a pipe/socket.
        let mut decoder = FrameDecoder::new();
        let mut got = Vec::new();
        for chunk in stream.chunks(1) {
            got.extend(decoder.push(chunk).expect("push"));
        }
        assert_eq!(decoder.pending_bytes(), 0);
        assert_eq!(got.len(), 2);
        assert_eq!(
            decode_envelope(&got[0]).expect("env").kind,
            kind::CLIENT_INPUT
        );
        assert_eq!(decode_envelope(&got[1]).expect("env").kind, kind::COMMAND);
    }

    #[test]
    fn oversize_length_prefix_is_rejected() {
        let mut decoder = FrameDecoder::new();
        let declared = (MAX_FRAME_BYTES + 1) as u32;
        let error = decoder
            .push(&declared.to_le_bytes())
            .expect_err("must reject");
        assert_eq!(
            error,
            CodecError::FrameTooLarge {
                declared: MAX_FRAME_BYTES + 1
            }
        );
    }

    #[test]
    fn foreign_version_is_rejected() {
        let envelope = Envelope {
            version: ProtocolVersion(999),
            kind: kind::HELLO,
            payload: Vec::new(),
        };
        let bytes = postcard::to_allocvec(&envelope).expect("encode");
        let error = decode_envelope(&bytes).expect_err("must reject");
        assert_eq!(error, CodecError::VersionMismatch { got: 999 });
    }
}
