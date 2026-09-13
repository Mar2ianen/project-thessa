//! Embedded server link: the client spawns `thessa-server` as a child
//! with piped stdio and flies on its snapshots instead of stepping locally.
//!
//! Layout mirrors the server: one reader thread owns the stdout pipe end
//! to end (Welcome gate first, then snapshots), one writer thread owns
//! stdin. The handshake runs on the calling thread with a timeout so a
//! missing or skewed server degrades to local simulation.

use std::io::{Read, Write};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Duration;

use thessa_flight_net::{ClientInput, Snapshot};
use thessa_protocol::{FrameDecoder, kind};

/// Live link to an embedded server child. Dropping kills the child.
#[derive(bevy::prelude::Resource)]
pub struct EmbeddedLink {
    // Receiver is !Sync; the frame loop only needs short locked drains.
    snapshots: std::sync::Mutex<Receiver<Snapshot>>,
    inputs: Sender<ClientInput>,
    child: Child,
}

impl Drop for EmbeddedLink {
    fn drop(&mut self) {
        let _ = self.child.kill();
    }
}

impl EmbeddedLink {
    /// Spawn the server next to the client binary, shake hands, return.
    /// Any failure (missing binary, timeout, version skew) is an `Err`
    /// so the caller falls back to local simulation.
    pub fn spawn(handshake_timeout: Duration) -> Result<Self, String> {
        let exe = std::env::current_exe().map_err(|e| e.to_string())?;
        let server = exe.with_file_name(format!("thessa-server{}", std::env::consts::EXE_SUFFIX));
        let mut child = Command::new(&server)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|e| format!("spawn {}: {e}", server.display()))?;

        let (snapshots_tx, snapshots) = channel();
        let (welcomed_tx, welcomed_rx) = channel::<()>();

        // Reader thread owns stdout end to end: Welcome gate, then snapshots.
        let mut child_out = child.stdout.take().ok_or("server stdout not piped")?;
        std::thread::spawn(move || {
            let mut decoder = FrameDecoder::new();
            let mut buffer = [0u8; 65536];
            let mut welcomed = false;
            loop {
                match child_out.read(&mut buffer) {
                    Ok(0) => break,
                    Ok(n) => match decoder.push(&buffer[..n]) {
                        Ok(frames) => {
                            for frame in frames {
                                let Ok(envelope) = thessa_flight_net::decode_frame(&frame) else {
                                    continue;
                                };
                                if !welcomed {
                                    if envelope.kind == kind::WELCOME {
                                        welcomed = true;
                                        let _ = welcomed_tx.send(());
                                    }
                                    continue;
                                }
                                if envelope.kind != kind::SNAPSHOT {
                                    continue;
                                }
                                if let Ok(snapshot) =
                                    thessa_flight_net::decode_payload::<Snapshot>(&envelope)
                                    && snapshots_tx.send(snapshot).is_err()
                                {
                                    return;
                                }
                            }
                        }
                        Err(error) => {
                            eprintln!("[client] frame error: {error}");
                            break;
                        }
                    },
                    Err(_) => break,
                }
            }
        });

        // Handshake on this thread: Hello out, Welcome (via the reader)
        // within the timeout. Stdin stays ours until the writer starts.
        let mut child_in = child.stdin.take().ok_or("server stdin not piped")?;
        let hello = thessa_flight_net::Hello {
            client_name: "thessa-client-embedded".into(),
        };
        let frame = thessa_flight_net::encode_hello(&hello).map_err(|e| e.to_string())?;
        child_in
            .write_all(&frame)
            .map_err(|e| format!("hello write: {e}"))?;
        child_in.flush().map_err(|e| format!("hello flush: {e}"))?;
        welcomed_rx
            .recv_timeout(handshake_timeout)
            .map_err(|_| "server handshake timed out".to_string())?;

        // Writer thread takes stdin from here; inputs are idempotent
        // snapshots of live control state, so no ordering hazards.
        let (inputs, inputs_rx): (Sender<ClientInput>, Receiver<ClientInput>) = channel();
        std::thread::spawn(move || {
            for input in inputs_rx {
                let Ok(frame) = thessa_flight_net::encode_input(&input) else {
                    continue;
                };
                if child_in.write_all(&frame).is_err() {
                    break;
                }
            }
            let _ = child_in.flush();
        });

        Ok(Self {
            snapshots: std::sync::Mutex::new(snapshots),
            inputs,
            child,
        })
    }

    /// Queue one input for the server (never blocks; drops on disconnect).
    pub fn send_input(&self, input: &ClientInput) {
        let _ = self.inputs.send(input.clone());
    }

    /// Drain snapshots, keeping the newest. `None` when nothing arrived.
    pub fn latest_snapshot(&self) -> Option<Snapshot> {
        let snapshots = self.snapshots.lock().ok()?;
        let mut latest = None;
        while let Ok(snapshot) = snapshots.try_recv() {
            latest = Some(snapshot);
        }
        latest
    }
}
