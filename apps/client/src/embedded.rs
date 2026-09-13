//! Embedded server link: the client spawns `thessa-server` as a child
//! with piped stdio and flies on its snapshots instead of stepping locally.
//!
//! Layout mirrors the server: one reader thread owns the stdout pipe end
//! to end (Welcome gate first, then snapshots), one writer thread owns
//! stdin. The handshake runs on the calling thread with a timeout so a
//! missing or skewed server degrades to local simulation.

use std::io::{Read, Write};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, channel};
use std::time::Duration;

use thessa_flight_net::{ClientInput, Snapshot};
use thessa_protocol::{FrameDecoder, kind};

struct ReceivedSnapshot {
    snapshot: Snapshot,
    received_at: std::time::Instant,
    generation: u64,
}

struct SnapshotInbox {
    receiver: Receiver<Snapshot>,
    previous: Option<ReceivedSnapshot>,
    latest: Option<ReceivedSnapshot>,
    next_generation: u64,
    delivered_generation: u64,
}

impl SnapshotInbox {
    fn drain(&mut self) {
        while let Ok(snapshot) = self.receiver.try_recv() {
            self.next_generation = self.next_generation.wrapping_add(1);
            let received = ReceivedSnapshot {
                snapshot,
                received_at: std::time::Instant::now(),
                generation: self.next_generation,
            };
            self.previous = self.latest.take();
            self.latest = Some(received);
        }
    }

    fn latest(&mut self) -> Option<Snapshot> {
        self.drain();
        let received = self.latest.as_ref()?;
        if received.generation == self.delivered_generation {
            return None;
        }
        self.delivered_generation = received.generation;
        Some(received.snapshot.clone())
    }

    /// Interpolate one wall-frame behind the newest received snapshot. The
    /// authoritative state is still returned by `latest`; this copy is only
    /// for presentation, so a future prediction can never become server truth.
    fn interpolated(&self) -> Option<Snapshot> {
        let latest = self.latest.as_ref()?;
        let Some(previous) = self.previous.as_ref() else {
            return Some(latest.snapshot.clone());
        };
        let interval = latest
            .received_at
            .duration_since(previous.received_at)
            .as_secs_f64();
        let alpha = if interval > 1.0e-6 {
            (latest.received_at.elapsed().as_secs_f64() / interval).clamp(0.0, 1.0)
        } else {
            1.0
        };
        Some(interpolate_snapshot(
            &previous.snapshot,
            &latest.snapshot,
            alpha,
        ))
    }
}

fn interpolate_snapshot(previous: &Snapshot, latest: &Snapshot, alpha: f64) -> Snapshot {
    let alpha = alpha.clamp(0.0, 1.0);
    let mut snapshot = latest.clone();
    snapshot.flight_time_s =
        previous.flight_time_s + (latest.flight_time_s - previous.flight_time_s) * alpha;
    snapshot.state.position_inertial_m = previous
        .state
        .position_inertial_m
        .lerp(latest.state.position_inertial_m, alpha);
    snapshot.state.velocity_inertial_mps = previous
        .state
        .velocity_inertial_mps
        .lerp(latest.state.velocity_inertial_mps, alpha);
    snapshot.state.orientation_body_to_inertial = previous
        .state
        .orientation_body_to_inertial
        .slerp(latest.state.orientation_body_to_inertial, alpha)
        .normalize();
    snapshot.state.angular_velocity_body_rps = previous
        .state
        .angular_velocity_body_rps
        .lerp(latest.state.angular_velocity_body_rps, alpha);
    snapshot
}

struct InputMailbox {
    pending: std::sync::Mutex<Option<ClientInput>>,
    wake: std::sync::Condvar,
    closed: AtomicBool,
}

impl InputMailbox {
    fn push(&self, input: ClientInput) {
        if self.closed.load(Ordering::Acquire) {
            return;
        }
        let mut pending = self.pending.lock().expect("input mailbox poisoned");
        if self.closed.load(Ordering::Acquire) {
            return;
        }
        if let Some(queued) = pending.as_mut() {
            let commands = std::mem::take(&mut queued.commands);
            *queued = input;
            queued.commands = merge_commands(commands, std::mem::take(&mut queued.commands));
        } else {
            *pending = Some(input);
        }
        self.wake.notify_one();
    }

    fn take(&self) -> Option<ClientInput> {
        let mut pending = self.pending.lock().expect("input mailbox poisoned");
        loop {
            if let Some(input) = pending.take() {
                return Some(input);
            }
            if self.closed.load(Ordering::Acquire) {
                return None;
            }
            pending = self.wake.wait(pending).expect("input mailbox poisoned");
        }
    }

    fn close(&self) {
        self.closed.store(true, Ordering::Release);
        self.wake.notify_all();
    }
}

fn merge_commands(
    mut queued: Vec<thessa_flight_net::Command>,
    incoming: Vec<thessa_flight_net::Command>,
) -> Vec<thessa_flight_net::Command> {
    for command in incoming {
        match command {
            thessa_flight_net::Command::SetWarp { .. } => {
                queued.retain(|old| !matches!(old, thessa_flight_net::Command::SetWarp { .. }));
                queued.push(command);
            }
            thessa_flight_net::Command::Pause { .. } => {
                queued.retain(|old| !matches!(old, thessa_flight_net::Command::Pause { .. }));
                queued.push(command);
            }
            // Stage and explicit engine commands are events; dropping one can
            // change the authoritative state, so retain their full order.
            event => queued.push(event),
        }
    }
    queued
}

/// Live link to an embedded server child. Dropping kills the child.
#[derive(bevy::prelude::Resource)]
pub struct EmbeddedLink {
    snapshots: std::sync::Mutex<SnapshotInbox>,
    inputs: std::sync::Arc<InputMailbox>,
    child: Child,
}

impl Drop for EmbeddedLink {
    fn drop(&mut self) {
        self.inputs.close();
        let _ = self.child.kill();
        let _ = self.child.wait();
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
        let inputs = std::sync::Arc::new(InputMailbox {
            pending: std::sync::Mutex::new(None),
            wake: std::sync::Condvar::new(),
            closed: AtomicBool::new(false),
        });
        let writer_inputs = inputs.clone();
        std::thread::spawn(move || {
            while let Some(input) = writer_inputs.take() {
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
            snapshots: std::sync::Mutex::new(SnapshotInbox {
                receiver: snapshots,
                previous: None,
                latest: None,
                next_generation: 0,
                delivered_generation: 0,
            }),
            inputs,
            child,
        })
    }

    /// Queue one input for the server (never blocks; drops on disconnect).
    pub fn send_input(&self, input: &ClientInput) {
        self.inputs.push(input.clone());
    }

    /// Drain snapshots, keeping the newest. `None` when nothing arrived.
    pub fn latest_snapshot(&self) -> Option<Snapshot> {
        self.snapshots.lock().ok()?.latest()
    }

    /// Presentation-only pose between the two newest authoritative snapshots.
    pub fn interpolated_snapshot(&self) -> Option<Snapshot> {
        self.snapshots.lock().ok()?.interpolated()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::{DQuat, DVec3};
    use thessa_flight_net::Command;
    use thessa_sim_core::RigidBodyState;

    fn snapshot(position: DVec3, angle: f64, time: f64) -> Snapshot {
        Snapshot {
            tick: time as u64,
            flight_time_s: time,
            state: RigidBodyState::new(
                position,
                DVec3::X,
                DQuat::from_rotation_z(angle),
                DVec3::ZERO,
            )
            .unwrap(),
            throttle: 0.0,
            engine_active: false,
            paused: false,
            effective_warp: 1.0,
            server_compute_s: 0.01,
            server_wall_s: 0.1,
            steps_this_frame: 1,
            rails_advanced_s: 0.0,
            wake_notice: None,
            flight_error: None,
        }
    }

    #[test]
    fn render_snapshot_interpolation_never_changes_authoritative_metadata() {
        let first = snapshot(DVec3::ZERO, 0.0, 10.0);
        let second = snapshot(DVec3::X * 10.0, std::f64::consts::PI, 20.0);
        let middle = interpolate_snapshot(&first, &second, 0.5);
        assert_eq!(middle.tick, second.tick);
        assert_eq!(middle.server_wall_s, second.server_wall_s);
        assert!((middle.flight_time_s - 15.0).abs() < 1.0e-12);
        assert!((middle.state.position_inertial_m - DVec3::X * 5.0).length() < 1.0e-12);
        assert!((middle.state.orientation_body_to_inertial.length() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn input_mailbox_merge_preserves_events_and_replaces_state_votes() {
        let merged = merge_commands(
            vec![Command::Stage, Command::Pause { paused: true }],
            vec![
                Command::Stage,
                Command::Pause { paused: false },
                Command::SetWarp { factor: 128.0 },
            ],
        );
        assert_eq!(merged[0], Command::Stage);
        assert_eq!(merged[1], Command::Stage);
        assert_eq!(merged[2], Command::Pause { paused: false });
        assert_eq!(merged[3], Command::SetWarp { factor: 128.0 });
    }

    #[test]
    fn snapshot_inbox_delivers_each_arrival_once() {
        let (sender, receiver) = channel();
        let mut inbox = SnapshotInbox {
            receiver,
            previous: None,
            latest: None,
            next_generation: 0,
            delivered_generation: 0,
        };
        sender.send(snapshot(DVec3::X, 0.0, 1.0)).expect("snapshot");
        assert!(inbox.latest().is_some());
        assert!(inbox.latest().is_none());
        sender
            .send(snapshot(DVec3::Y, 0.0, 1.0))
            .expect("second snapshot");
        assert_eq!(
            inbox
                .latest()
                .expect("new snapshot")
                .state
                .position_inertial_m,
            DVec3::Y
        );
        assert!(inbox.latest().is_none());
    }

    #[test]
    fn input_mailbox_close_wakes_waiters() {
        let mailbox = std::sync::Arc::new(InputMailbox {
            pending: std::sync::Mutex::new(None),
            wake: std::sync::Condvar::new(),
            closed: AtomicBool::new(false),
        });
        let waiter = mailbox.clone();
        let thread = std::thread::spawn(move || waiter.take());
        mailbox.close();
        assert!(thread.join().expect("waiter thread").is_none());
    }
}
