//! Embedded server link: the client spawns `thessa-server` as a child
//! with piped stdio and flies on its snapshots instead of stepping locally.
//!
//! Layout mirrors the server: one reader thread owns the stdout pipe end
//! to end (Welcome gate first, then snapshots), one writer thread owns
//! stdin. The handshake runs on the calling thread with a timeout so a
//! missing or skewed server degrades to local simulation.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use thessa_flight_net::{ClientInput, Snapshot};
use thessa_protocol::{FrameDecoder, kind};

struct ReceivedSnapshot {
    snapshot: Snapshot,
    received_at: std::time::Instant,
    generation: u64,
}

struct SnapshotInbox {
    queue: Arc<Mutex<VecDeque<ReceivedSnapshot>>>,
    previous: Option<ReceivedSnapshot>,
    latest: Option<ReceivedSnapshot>,
    next_generation: u64,
    delivered_generation: u64,
}

#[derive(Clone)]
struct SnapshotInboxWriter {
    queue: Arc<Mutex<VecDeque<ReceivedSnapshot>>>,
}

impl SnapshotInbox {
    fn new() -> (Self, SnapshotInboxWriter) {
        let queue = Arc::new(Mutex::new(VecDeque::with_capacity(2)));
        (
            Self {
                queue: queue.clone(),
                previous: None,
                latest: None,
                next_generation: 0,
                delivered_generation: 0,
            },
            SnapshotInboxWriter { queue },
        )
    }

    fn drain(&mut self) {
        // A poisoned inbox means the reader thread panicked: keep serving
        // the last good snapshots instead of killing the render thread.
        let Ok(mut queue) = self.queue.lock() else {
            return;
        };
        let received_snapshots = queue.drain(..).collect::<Vec<_>>();
        drop(queue);
        for received in received_snapshots {
            self.next_generation = self.next_generation.wrapping_add(1);
            let received = ReceivedSnapshot {
                generation: self.next_generation,
                ..received
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

impl SnapshotInboxWriter {
    /// Non-blocking latest-wins ingress. At most the two newest
    /// receive-timestamped snapshots are retained for interpolation.
    fn push(&self, received: ReceivedSnapshot) -> bool {
        let Ok(mut queue) = self.queue.try_lock() else {
            return false;
        };
        if queue.len() == 2 {
            queue.pop_front();
        }
        queue.push_back(received);
        true
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

const MAX_PENDING_EDGE_COMMANDS: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputPushError {
    Closed,
    TooManyEdgeCommands,
}

impl InputMailbox {
    fn push(&self, input: ClientInput) -> Result<(), InputPushError> {
        if self.closed.load(Ordering::Acquire) {
            return Err(InputPushError::Closed);
        }
        // Poisoning means another thread panicked while holding the mailbox:
        // the link is unusable, report it as closed instead of panicking.
        let Ok(mut pending) = self.pending.lock() else {
            return Err(InputPushError::Closed);
        };
        if self.closed.load(Ordering::Acquire) {
            return Err(InputPushError::Closed);
        }
        let mut input = input;
        let queued_commands = pending
            .as_mut()
            .map(|queued| std::mem::take(&mut queued.commands))
            .unwrap_or_default();
        let original_commands = queued_commands.clone();
        let merged_commands = merge_commands(queued_commands, std::mem::take(&mut input.commands));
        if merged_commands
            .iter()
            .filter(|command| is_edge_command(command))
            .count()
            > MAX_PENDING_EDGE_COMMANDS
        {
            if let Some(queued) = pending.as_mut() {
                queued.commands = original_commands;
            }
            return Err(InputPushError::TooManyEdgeCommands);
        }
        input.commands = merged_commands;
        *pending = Some(input);
        self.wake.notify_one();
        Ok(())
    }

    fn take(&self) -> Option<ClientInput> {
        let Ok(mut pending) = self.pending.lock() else {
            return None;
        };
        loop {
            if let Some(input) = pending.take() {
                return Some(input);
            }
            if self.closed.load(Ordering::Acquire) {
                return None;
            }
            pending = match self.wake.wait(pending) {
                Ok(guard) => guard,
                Err(_) => return None,
            };
        }
    }

    fn close(&self) {
        // `take` checks `closed` while holding this mutex and then waits on
        // the same predicate. Change the predicate under that mutex so a
        // waiter cannot observe the old value, release the lock, and miss
        // the notification between its check and wait. If the mutex is
        // poisoned the waiter is already broken: still flip the flag.
        let _guard = self.pending.lock();
        self.closed.store(true, Ordering::Release);
        self.wake.notify_all();
    }
}

fn is_edge_command(command: &thessa_flight_net::Command) -> bool {
    matches!(
        command,
        thessa_flight_net::Command::Stage
            | thessa_flight_net::Command::Engine { .. }
            | thessa_flight_net::Command::Reset
    )
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

/// Owns a child during the fallible startup/handshake phase. `Child` itself
/// does not kill or reap the process when dropped, so every startup error
/// must pass through this guard.
struct StartupChild {
    child: Option<Child>,
}

impl StartupChild {
    fn new(child: Child) -> Self {
        Self { child: Some(child) }
    }

    fn as_mut(&mut self) -> &mut Child {
        self.child.as_mut().expect("startup child already disarmed")
    }

    fn disarm(mut self) -> Child {
        self.child.take().expect("startup child already disarmed")
    }
}

impl Drop for StartupChild {
    fn drop(&mut self) {
        if let Some(child) = self.child.as_mut() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

impl EmbeddedLink {
    /// Spawn the server next to the client binary, shake hands, return.
    /// Any failure (missing binary, timeout, version skew) is an `Err`
    /// so the caller falls back to local simulation.
    pub fn spawn(handshake_timeout: Duration) -> Result<Self, String> {
        let exe = std::env::current_exe().map_err(|e| e.to_string())?;
        let server = exe.with_file_name(format!("thessa-server{}", std::env::consts::EXE_SUFFIX));
        let mut command = Command::new(&server);
        command
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit());
        Self::spawn_command(command, handshake_timeout)
            .map_err(|error| format!("spawn {}: {error}", server.display()))
    }

    fn spawn_command(mut command: Command, handshake_timeout: Duration) -> Result<Self, String> {
        let child = command
            .spawn()
            .map_err(|error| format!("child process: {error}"))?;
        let mut startup = StartupChild::new(child);

        let (snapshots, snapshots_writer) = SnapshotInbox::new();
        let (welcomed_tx, welcomed_rx) = channel::<()>();

        // Reader thread owns stdout end to end: Welcome gate, then snapshots.
        let mut child_out = startup
            .as_mut()
            .stdout
            .take()
            .ok_or("server stdout not piped")?;
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
                                {
                                    let accepted = snapshots_writer.push(ReceivedSnapshot {
                                        snapshot,
                                        received_at: std::time::Instant::now(),
                                        generation: 0,
                                    });
                                    if !accepted {
                                        eprintln!(
                                            "[client] snapshot inbox busy; dropping stale snapshot"
                                        );
                                    }
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
        let mut child_in = startup
            .as_mut()
            .stdin
            .take()
            .ok_or("server stdin not piped")?;
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

        let child = startup.disarm();

        Ok(Self {
            snapshots: std::sync::Mutex::new(snapshots),
            inputs,
            child,
        })
    }
    /// Queue one input for the server (never blocks; closes on command
    /// overflow or disconnect so an accepted edge can never be silently
    /// dropped).
    pub fn send_input(&self, input: &ClientInput) -> bool {
        match self.inputs.push(input.clone()) {
            Ok(()) => true,
            Err(InputPushError::Closed) => false,
            Err(InputPushError::TooManyEdgeCommands) => {
                eprintln!(
                    "[client] embedded input mailbox overflow; closing link to preserve command ordering"
                );
                self.inputs.close();
                false
            }
        }
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
        let (mut inbox, sender) = SnapshotInbox::new();
        assert!(sender.push(ReceivedSnapshot {
            snapshot: snapshot(DVec3::X, 0.0, 1.0),
            received_at: std::time::Instant::now(),
            generation: 0,
        }));
        assert!(inbox.latest().is_some());
        assert!(inbox.latest().is_none());
        assert!(sender.push(ReceivedSnapshot {
            snapshot: snapshot(DVec3::Y, 0.0, 1.0),
            received_at: std::time::Instant::now(),
            generation: 0,
        }));
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
    fn snapshot_inbox_preserves_reader_receive_times_when_drained_together() {
        let (mut inbox, sender) = SnapshotInbox::new();
        let base = std::time::Instant::now();
        assert!(sender.push(ReceivedSnapshot {
            snapshot: snapshot(DVec3::X, 0.0, 1.0),
            received_at: base,
            generation: 0,
        }));
        assert!(sender.push(ReceivedSnapshot {
            snapshot: snapshot(DVec3::Y, 0.0, 2.0),
            received_at: base + Duration::from_millis(50),
            generation: 0,
        }));

        assert!(
            inbox.latest().is_some(),
            "drain should accept both arrivals"
        );
        let previous = inbox.previous.as_ref().expect("previous snapshot");
        let latest = inbox.latest.as_ref().expect("latest snapshot");
        assert_eq!(
            latest.received_at.duration_since(previous.received_at),
            Duration::from_millis(50)
        );
    }

    #[test]
    fn snapshot_inbox_flood_keeps_only_the_two_newest_arrivals() {
        let (mut inbox, sender) = SnapshotInbox::new();
        for position in 0..100 {
            assert!(sender.push(ReceivedSnapshot {
                snapshot: snapshot(DVec3::X * position as f64, 0.0, position as f64),
                received_at: std::time::Instant::now(),
                generation: 0,
            }));
        }
        inbox.drain();
        assert_eq!(inbox.previous.as_ref().unwrap().snapshot.tick, 98);
        assert_eq!(inbox.latest.as_ref().unwrap().snapshot.tick, 99);
    }

    #[test]
    fn input_mailbox_close_wakes_waiters() {
        let mailbox = std::sync::Arc::new(InputMailbox {
            pending: std::sync::Mutex::new(None),
            wake: std::sync::Condvar::new(),
            closed: AtomicBool::new(false),
        });
        let waiter = mailbox.clone();
        let (done_tx, done_rx) = channel();
        let thread = std::thread::spawn(move || {
            done_tx
                .send(waiter.take().is_none())
                .expect("waiter result");
        });
        mailbox.close();
        assert!(
            done_rx
                .recv_timeout(Duration::from_millis(250))
                .expect("close must wake a blocked waiter")
        );
        thread.join().expect("waiter thread");
    }

    #[test]
    fn input_mailbox_bounds_edge_events_without_dropping_an_accepted_input() {
        let mailbox = InputMailbox {
            pending: std::sync::Mutex::new(None),
            wake: std::sync::Condvar::new(),
            closed: AtomicBool::new(false),
        };
        let mut input = ClientInput {
            tick: 0,
            control_input: [0.0; 3],
            control_mode: thessa_flight_authority::ControlMode::Direct,
            sas_target_xyzw: [0.0, 0.0, 0.0, 1.0],
            throttle: 0.0,
            engine_active: false,
            sas_enabled: false,
            rcs_enabled: false,
            gear_down: false,
            commands: vec![Command::Stage; MAX_PENDING_EDGE_COMMANDS],
        };
        assert!(mailbox.push(input.clone()).is_ok());
        input.commands.push(Command::Stage);
        assert_eq!(
            mailbox.push(input),
            Err(InputPushError::TooManyEdgeCommands)
        );
        assert_eq!(
            mailbox.take().expect("accepted input").commands.len(),
            MAX_PENDING_EDGE_COMMANDS
        );
    }

    #[cfg(unix)]
    fn injected_server(command: &str) -> (std::process::Command, std::path::PathBuf) {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock")
            .as_nanos();
        let pid_path = std::env::temp_dir().join(format!(
            "thessa-embedded-startup-{}-{stamp}.pid",
            std::process::id()
        ));
        let quoted_path = format!("'{}'", pid_path.to_string_lossy().replace('\'', "'\\''"));
        let script = format!("echo $$ > {quoted_path}; {command}");
        let mut command_builder = std::process::Command::new("sh");
        command_builder
            .args(["-c", script.as_str()])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null());
        (command_builder, pid_path)
    }

    #[cfg(unix)]
    fn assert_startup_child_gone(pid_path: &std::path::Path) {
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        let pid = loop {
            if let Ok(pid) = std::fs::read_to_string(pid_path)
                && let Ok(pid) = pid.trim().parse::<u32>()
            {
                break pid;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "startup fixture did not publish its pid"
            );
            std::thread::sleep(Duration::from_millis(1));
        };
        let status = std::process::Command::new("kill")
            .args(["-0", &pid.to_string()])
            .stderr(Stdio::null())
            .status()
            .expect("run kill -0");
        assert!(!status.success(), "startup child {pid} is still alive");
        #[cfg(target_os = "linux")]
        assert!(
            !std::path::Path::new("/proc").join(pid.to_string()).exists(),
            "startup child {pid} was not reaped"
        );
        let _ = std::fs::remove_file(pid_path);
    }

    #[cfg(unix)]
    #[test]
    fn embedded_startup_timeout_reaps_child() {
        let started = std::time::Instant::now();
        let (command, pid_path) = injected_server("exec sleep 30");
        let result = EmbeddedLink::spawn_command(command, Duration::from_millis(25));
        assert!(result.is_err(), "server without welcome must fail startup");
        assert_startup_child_gone(&pid_path);
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "startup cleanup took too long: {:?}",
            started.elapsed()
        );
    }

    #[cfg(unix)]
    #[test]
    fn embedded_startup_early_exit_reaps_child() {
        let started = std::time::Instant::now();
        let (command, pid_path) = injected_server("exit 0");
        let result = EmbeddedLink::spawn_command(command, Duration::from_millis(250));
        assert!(result.is_err(), "early server exit must fail startup");
        assert_startup_child_gone(&pid_path);
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "early-exit cleanup took too long: {:?}",
            started.elapsed()
        );
    }
}
