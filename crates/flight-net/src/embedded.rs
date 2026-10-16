//! Native embedded server link: a host spawns `thessa-server` as a child
//! with piped stdio and flies on its snapshots instead of stepping locally.
//!
//! Layout mirrors the server: one reader thread owns the stdout pipe end
//! to end (Welcome gate first, then snapshots), one writer thread owns
//! stdin. The reader also receives bounded on-change fleet geometry catalogues.
//! The handshake runs on the calling thread with a timeout.
//! The host chooses whether to fall back. This transport neither steps a
//! local authority nor adopts wire state into simulation/render state.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::presentation::interpolate_snapshot;
use crate::{ClientInput, FleetSnapshot, Snapshot};
use thessa_protocol::{FrameDecoder, kind};

struct ReceivedSnapshot<T = Snapshot> {
    snapshot: T,
    received_at: std::time::Instant,
    generation: u64,
}

struct SnapshotInbox<T = Snapshot> {
    queue: Arc<Mutex<VecDeque<ReceivedSnapshot<T>>>>,
    previous: Option<ReceivedSnapshot<T>>,
    latest: Option<ReceivedSnapshot<T>>,
    next_generation: u64,
    delivered_generation: u64,
}

#[derive(Clone)]
struct SnapshotInboxWriter<T = Snapshot> {
    queue: Arc<Mutex<VecDeque<ReceivedSnapshot<T>>>>,
}

impl<T: Clone> SnapshotInbox<T> {
    fn new() -> (Self, SnapshotInboxWriter<T>) {
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

    fn latest(&mut self) -> Option<T> {
        self.drain();
        let received = self.latest.as_ref()?;
        if received.generation == self.delivered_generation {
            return None;
        }
        self.delivered_generation = received.generation;
        Some(received.snapshot.clone())
    }

    fn age(&self) -> Option<Duration> {
        Some(self.latest.as_ref()?.received_at.elapsed())
    }
}

impl SnapshotInbox<Snapshot> {
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

impl<T> SnapshotInboxWriter<T> {
    /// Non-blocking latest-wins ingress. At most the two newest
    /// receive-timestamped snapshots are retained for interpolation.
    fn push(&self, received: ReceivedSnapshot<T>) -> bool {
        let Ok(mut queue) = self.queue.try_lock() else {
            return false;
        };
        Self::retain_latest(&mut queue, received);
        true
    }

    /// A catalogue is sent on change, not periodically: mutex contention
    /// must not silently lose its only publication. Only the reader waits on
    /// this short bounded queue lock; it never holds a lock during pipe IO.
    fn push_catalogue(&self, received: ReceivedSnapshot<T>) -> bool {
        let Ok(mut queue) = self.queue.lock() else {
            return false;
        };
        Self::retain_latest(&mut queue, received);
        true
    }

    fn retain_latest(queue: &mut VecDeque<ReceivedSnapshot<T>>, received: ReceivedSnapshot<T>) {
        if queue.len() == 2 {
            queue.pop_front();
        }
        queue.push_back(received);
    }
}

struct InputMailbox {
    pending: std::sync::Mutex<Option<ClientInput>>,
    wake: std::sync::Condvar,
    closed: AtomicBool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputPushError {
    Closed,
    TooManyCommands,
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
        // The server validates the whole packet, including state votes.
        // Count every variant so newer fleet/autopilot events cannot bypass
        // the retained queue bound or make an accepted packet unsendable.
        if merged_commands.len() > crate::MAX_COMMANDS_PER_INPUT {
            if let Some(queued) = pending.as_mut() {
                queued.commands = original_commands;
            }
            return Err(InputPushError::TooManyCommands);
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

fn merge_commands(
    mut queued: Vec<crate::Command>,
    incoming: Vec<crate::Command>,
) -> Vec<crate::Command> {
    for command in incoming {
        match command {
            crate::Command::SetWarp { .. } => {
                queued.retain(|old| !matches!(old, crate::Command::SetWarp { .. }));
                queued.push(command);
            }
            crate::Command::Pause { .. } => {
                queued.retain(|old| !matches!(old, crate::Command::Pause { .. }));
                queued.push(command);
            }
            // Stage, part, and explicit engine commands are events; dropping
            // one can change authoritative state, so retain their full order.
            event => queued.push(event),
        }
    }
    queued
}

/// Live link to an embedded server child. Dropping kills the child.
pub struct EmbeddedLink {
    snapshots: std::sync::Mutex<SnapshotInbox>,
    fleets: std::sync::Mutex<SnapshotInbox<FleetSnapshot>>,
    geometries: std::sync::Mutex<SnapshotInbox<Arc<crate::geometry::FleetGeometry>>>,
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
    /// Any failure (missing binary, timeout, version skew) is an `Err`;
    /// fallback or fail-fast behavior belongs to the caller.
    pub fn spawn(handshake_timeout: Duration) -> Result<Self, String> {
        let exe = std::env::current_exe().map_err(|e| e.to_string())?;
        let server = exe.with_file_name(format!("thessa-server{}", std::env::consts::EXE_SUFFIX));
        Self::spawn_server(&server, handshake_timeout)
    }

    /// Spawn a caller-selected native server executable with the same stdio
    /// protocol and lifecycle as [`Self::spawn`]. No shell or additional args.
    pub fn spawn_server(
        server: &std::path::Path,
        handshake_timeout: Duration,
    ) -> Result<Self, String> {
        let mut command = Command::new(server);
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
        let (fleets, fleets_writer) = SnapshotInbox::new();
        let (geometries, geometries_writer) = SnapshotInbox::new();
        let (welcomed_tx, welcomed_rx) = channel::<()>();
        let inputs = std::sync::Arc::new(InputMailbox {
            pending: std::sync::Mutex::new(None),
            wake: std::sync::Condvar::new(),
            closed: AtomicBool::new(false),
        });
        let reader_inputs = inputs.clone();

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
            'read: loop {
                match child_out.read(&mut buffer) {
                    Ok(0) => break,
                    Ok(n) => match decoder.push(&buffer[..n]) {
                        Ok(frames) => {
                            for frame in frames {
                                let Ok(envelope) = crate::decode_frame(&frame) else {
                                    continue;
                                };
                                if !welcomed {
                                    if envelope.kind == kind::WELCOME {
                                        welcomed = true;
                                        let _ = welcomed_tx.send(());
                                    }
                                    continue;
                                }
                                if envelope.kind == kind::FLEET_GEOMETRY {
                                    match crate::decode_payload::<crate::geometry::FleetGeometry>(
                                        &envelope,
                                    ) {
                                        Ok(geometry) => {
                                            if let Err(error) = geometry.validate() {
                                                eprintln!(
                                                    "[client] invalid fleet geometry: {error}"
                                                );
                                                break 'read;
                                            }
                                            if !geometries_writer.push_catalogue(ReceivedSnapshot {
                                                snapshot: Arc::new(geometry),
                                                received_at: std::time::Instant::now(),
                                                generation: 0,
                                            }) {
                                                break 'read;
                                            }
                                        }
                                        Err(error) => {
                                            eprintln!(
                                                "[client] invalid fleet geometry payload: {error}"
                                            );
                                            break 'read;
                                        }
                                    }
                                    continue;
                                }
                                if envelope.kind == kind::FLEET_SNAPSHOT {
                                    match crate::decode_payload::<FleetSnapshot>(&envelope) {
                                        Ok(fleet) => {
                                            if let Err(error) = fleet.validate_membership() {
                                                eprintln!("[client] invalid fleet: {error}");
                                                break 'read;
                                            }
                                            fleets_writer.push(ReceivedSnapshot {
                                                snapshot: fleet,
                                                received_at: std::time::Instant::now(),
                                                generation: 0,
                                            });
                                        }
                                        Err(error) => {
                                            eprintln!("[client] invalid fleet payload: {error}");
                                            break 'read;
                                        }
                                    }
                                    continue;
                                }
                                if envelope.kind != kind::SNAPSHOT {
                                    continue;
                                }
                                if let Ok(snapshot) = crate::decode_payload::<Snapshot>(&envelope) {
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
            // EOF, read failure or framing failure ends the receive half.
            // Surface that state to the host and wake an idle writer.
            reader_inputs.close();
        });

        // Handshake on this thread: Hello out, Welcome (via the reader)
        // within the timeout. Stdin stays ours until the writer starts.
        let mut child_in = startup
            .as_mut()
            .stdin
            .take()
            .ok_or("server stdin not piped")?;
        let hello = crate::Hello {
            client_name: "thessa-client-embedded".into(),
        };
        let frame = crate::encode_hello(&hello).map_err(|e| e.to_string())?;
        child_in
            .write_all(&frame)
            .map_err(|e| format!("hello write: {e}"))?;
        child_in.flush().map_err(|e| format!("hello flush: {e}"))?;
        welcomed_rx
            .recv_timeout(handshake_timeout)
            .map_err(|_| "server handshake timed out".to_string())?;

        // Writer thread takes stdin from here; state intents coalesce but
        // discrete commands retain their full order in the bounded mailbox.
        let writer_inputs = inputs.clone();
        std::thread::spawn(move || {
            while let Some(input) = writer_inputs.take() {
                let Ok(frame) = crate::encode_input(&input) else {
                    break;
                };
                if child_in.write_all(&frame).is_err() {
                    break;
                }
            }
            writer_inputs.close();
            let _ = child_in.flush();
        });

        let child = startup.disarm();

        Ok(Self {
            snapshots: std::sync::Mutex::new(snapshots),
            fleets: std::sync::Mutex::new(fleets),
            geometries: std::sync::Mutex::new(geometries),
            inputs,
            child,
        })
    }
    /// Queue one input for the server (briefly locks the mailbox). An overflow
    /// or observed pipe/encode failure closes the link rather than silently
    /// accepting further intent. `true` means queued, not server acknowledged.
    pub fn send_input(&self, input: &ClientInput) -> bool {
        // Reject an oversized caller packet before cloning its command list.
        if input.commands.len() > crate::MAX_COMMANDS_PER_INPUT {
            self.inputs.close();
            return false;
        }
        match self.inputs.push(input.clone()) {
            Ok(()) => true,
            Err(InputPushError::Closed) => false,
            Err(InputPushError::TooManyCommands) => {
                eprintln!(
                    "[client] embedded input mailbox overflow; closing link to preserve command ordering"
                );
                self.inputs.close();
                false
            }
        }
    }

    /// A worker observed EOF/error, or the host overflowed/closed the mailbox.
    /// This is not a heartbeat: a live but stalled peer needs host timeouts.
    pub fn is_closed(&self) -> bool {
        self.inputs.closed.load(Ordering::Acquire)
    }

    /// Drain snapshots, keeping the newest. `None` when nothing arrived.
    pub fn latest_snapshot(&self) -> Option<Snapshot> {
        self.snapshots.lock().ok()?.latest()
    }

    /// Age of the newest drained snapshot using the reader's receive timestamp,
    /// not host polling or simulation time. Call `latest_snapshot` first to drain.
    pub fn snapshot_age(&self) -> Option<Duration> {
        self.snapshots.lock().ok()?.age()
    }

    /// Complete fleet membership, once per arrival; independent of PRIMARY's
    /// generation. Empty is explicit removal, `None` means no new packet.
    /// Same reader and bounded two-arrival inbox as PRIMARY, no extra worker.
    pub fn latest_fleet_snapshot(&self) -> Option<FleetSnapshot> {
        self.fleets.lock().ok()?.latest()
    }

    /// Immutable complete catalogue, independently latest-wins. Its revision
    /// must match a FleetSnapshot before use; no periodic geometry cloning.
    pub fn latest_fleet_geometry(&self) -> Option<Arc<crate::geometry::FleetGeometry>> {
        self.geometries.lock().ok()?.latest()
    }

    /// Reader age of the newest drained fleet packet; not PRIMARY's age and
    /// not a guarantee of per-vehicle progress. Drain with latest_fleet_snapshot.
    pub fn fleet_snapshot_age(&self) -> Option<Duration> {
        self.fleets.lock().ok()?.age()
    }

    /// Presentation-only pose between the two newest authoritative snapshots.
    pub fn interpolated_snapshot(&self) -> Option<Snapshot> {
        self.snapshots.lock().ok()?.interpolated()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Command;
    use glam::{DQuat, DVec3};
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
            reaction_wheel_torque_body_nm: [0.0; 3],
            parachutes: Vec::new(),
            wake_notice: None,
            flight_error: None,
        }
    }

    fn input(commands: Vec<Command>) -> ClientInput {
        ClientInput {
            tick: 0,
            vehicle_id: thessa_sim_core::VehicleId::PRIMARY,
            control_input: [0.0; 3],
            control_mode: thessa_flight_authority::ControlMode::Direct,
            sas_target_xyzw: [0.0, 0.0, 0.0, 1.0],
            throttle: 0.0,
            engine_active: false,
            sas_enabled: false,
            rcs_enabled: false,
            reaction_wheels_enabled: true,
            gear_down: false,
            parachutes_armed: false,
            commands,
        }
    }

    fn fleet(ids: impl IntoIterator<Item = u32>) -> FleetSnapshot {
        FleetSnapshot {
            geometry_revision: 1,
            vehicles: ids
                .into_iter()
                .map(|id| crate::FleetVehicleSnapshot {
                    vehicle_id: thessa_sim_core::VehicleId::new(id),
                    snapshot: snapshot(DVec3::X, 0.0, 1.0),
                })
                .collect(),
        }
    }

    #[test]
    fn fleet_membership_bound_ids_and_empty_generation_are_explicit() {
        for invalid in [fleet([0]), fleet([1, 1]), fleet([2, 1]), fleet(1..=64)] {
            assert!(invalid.validate_membership().is_err());
        }
        fleet(1..64).validate_membership().unwrap();
        let (mut inbox, writer) = SnapshotInbox::<FleetSnapshot>::new();
        assert!(inbox.latest().is_none());
        assert!(inbox.age().is_none());
        for ids in [vec![1, 2], vec![]] {
            let sample = fleet(ids);
            sample.validate_membership().unwrap();
            assert!(writer.push(ReceivedSnapshot {
                snapshot: sample.clone(),
                received_at: std::time::Instant::now() - Duration::from_millis(100),
                generation: 0
            }));
            assert_eq!(inbox.latest(), Some(sample));
            assert!(inbox.latest().is_none());
            assert!(inbox.age().unwrap() >= Duration::from_millis(100));
        }
        // Flood stays bounded by the same two-arrival mechanism as PRIMARY.
        for id in 1..100 {
            writer.push(ReceivedSnapshot {
                snapshot: fleet([id]),
                received_at: std::time::Instant::now(),
                generation: 0,
            });
        }
        inbox.drain();
        assert_eq!(
            inbox.previous.as_ref().unwrap().snapshot.vehicles[0]
                .vehicle_id
                .0,
            98
        );
        assert_eq!(
            inbox.latest.as_ref().unwrap().snapshot.vehicles[0]
                .vehicle_id
                .0,
            99
        );
    }

    #[test]
    fn on_change_catalogue_publication_survives_queue_contention() {
        let (mut inbox, writer) = SnapshotInbox::<u64>::new();
        let queue = inbox.queue.clone();
        let held = queue.lock().unwrap();
        let (started_tx, started_rx) = std::sync::mpsc::channel();
        let worker = std::thread::spawn(move || {
            started_tx.send(()).unwrap();
            writer.push_catalogue(ReceivedSnapshot {
                snapshot: 42,
                received_at: std::time::Instant::now(),
                generation: 0,
            })
        });
        started_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        drop(held);
        assert!(worker.join().unwrap());
        assert_eq!(inbox.latest(), Some(42));
        assert!(inbox.latest().is_none());
    }

    #[cfg(unix)]
    #[test]
    fn one_reader_demultiplexes_fleet_without_consuming_primary_generation() {
        let primary = snapshot(DVec3::Y, 0.0, 3.0);
        let fleet = fleet([1, 7]);
        let catalogue = crate::geometry::FleetGeometry {
            revision: 1,
            vehicles: vec![],
        };
        let wire: String = crate::encode_snapshot(&primary)
            .unwrap()
            .into_iter()
            .chain(crate::encode_fleet_snapshot(&fleet).unwrap())
            .chain(crate::geometry::encode_fleet_geometry(&catalogue).unwrap())
            .map(|byte| format!("\\{byte:03o}"))
            .collect();
        let (command, pid) = welcomed_server(&format!("printf '{wire}'; exec sleep 30"));
        let link = EmbeddedLink::spawn_command(command, Duration::from_secs(5)).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let received = loop {
            if let Some(fleet) = link.latest_fleet_snapshot() {
                break fleet;
            }
            assert!(!link.is_closed());
            assert!(std::time::Instant::now() < deadline);
            std::thread::yield_now();
        };
        assert_eq!(received, fleet);
        let received_geometry = loop {
            if let Some(g) = link.latest_fleet_geometry() {
                break g;
            }
            assert!(std::time::Instant::now() < deadline);
            std::thread::yield_now();
        };
        assert_eq!(*received_geometry, catalogue);
        assert!(link.latest_fleet_geometry().is_none());
        assert_eq!(link.latest_snapshot(), Some(primary));
        assert!(link.latest_snapshot().is_none());
        assert!(link.latest_fleet_snapshot().is_none());
        assert!(link.fleet_snapshot_age().is_some());
        drop(link);
        assert_startup_child_gone(&pid);
    }

    #[cfg(unix)]
    #[test]
    fn invalid_fleet_membership_closes_the_shared_link() {
        for wire in [
            crate::encode_fleet_snapshot(&fleet([1, 1])).unwrap(),
            crate::encode_frame(kind::FLEET_SNAPSHOT, &()).unwrap(),
            crate::encode_frame(kind::FLEET_GEOMETRY, &()).unwrap(),
            crate::geometry::encode_fleet_geometry(&crate::geometry::FleetGeometry {
                revision: 0,
                vehicles: vec![],
            })
            .unwrap(),
        ] {
            let bytes: String = wire.iter().map(|byte| format!("\\{byte:03o}")).collect();
            let (command, pid) = welcomed_server(&format!("printf '{bytes}'; exec sleep 30"));
            let link = EmbeddedLink::spawn_command(command, Duration::from_secs(5)).unwrap();
            wait_closed(&link);
            assert!(link.latest_fleet_snapshot().is_none());
            drop(link);
            assert_startup_child_gone(&pid);
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
        let mut input = input(vec![Command::Stage; crate::MAX_COMMANDS_PER_INPUT]);
        assert!(mailbox.push(input.clone()).is_ok());
        input.commands.push(Command::Part {
            command: thessa_sim_core::VehiclePartCommand::SetRcsEnabled { enabled: true },
        });
        assert_eq!(mailbox.push(input), Err(InputPushError::TooManyCommands));
        assert_eq!(
            mailbox.take().expect("accepted input").commands.len(),
            crate::MAX_COMMANDS_PER_INPUT
        );
    }

    #[test]
    fn fleet_events_and_state_votes_share_the_wire_command_bound() {
        let mailbox = InputMailbox {
            pending: std::sync::Mutex::new(None),
            wake: std::sync::Condvar::new(),
            closed: AtomicBool::new(false),
        };
        let event = Command::Separate {
            vehicle_id: thessa_sim_core::VehicleId::PRIMARY,
            link_name: "stage-link".into(),
        };
        let mut accepted = input(vec![event.clone(); crate::MAX_COMMANDS_PER_INPUT - 2]);
        accepted.commands.extend([
            Command::Pause { paused: true },
            Command::SetWarp { factor: 1.0 },
        ]);
        accepted.validate().unwrap();
        mailbox.push(accepted.clone()).unwrap();
        let votes = input(vec![
            Command::Pause { paused: true },
            Command::SetWarp { factor: 1.0 },
        ]);
        mailbox.push(votes).unwrap();
        let mut incoming = input(vec![event, Command::Pause { paused: false }]);
        incoming.tick = 99;
        assert_eq!(mailbox.push(incoming), Err(InputPushError::TooManyCommands));
        let retained = mailbox.take().unwrap();
        retained.validate().unwrap();
        assert_eq!(retained.commands, accepted.commands);
        assert_eq!(retained.tick, accepted.tick);
    }

    #[cfg(unix)]
    fn injected_server(command: &str) -> (std::process::Command, std::path::PathBuf) {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock")
            .as_nanos();
        // Keep scratch local to this crate/worktree, not the shared TMPDIR
        // used by concurrent builds and test processes on this machine.
        let scratch = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp");
        std::fs::create_dir_all(&scratch).expect("create embedded fixture scratch directory");
        let pid_path = scratch.join(format!(
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
            .stderr(Stdio::inherit());
        (command_builder, pid_path)
    }

    #[cfg(unix)]
    fn assert_startup_child_gone(pid_path: &std::path::Path) {
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let pid = loop {
            if let Ok(pid) = std::fs::read_to_string(pid_path)
                && let Ok(pid) = pid.trim().parse::<u32>()
            {
                break pid;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "startup fixture did not publish its pid at {}",
                pid_path.display()
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
        // Give the shell a scheduling window under a parallel workspace
        // run before timeout cleanup can kill it; still far below sleep 30.
        let result = EmbeddedLink::spawn_command(command, Duration::from_millis(250));
        assert!(result.is_err(), "server without welcome must fail startup");
        assert_startup_child_gone(&pid_path);
        assert!(
            started.elapsed() < Duration::from_secs(5),
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
            started.elapsed() < Duration::from_secs(5),
            "early-exit cleanup took too long: {:?}",
            started.elapsed()
        );
    }

    #[cfg(unix)]
    #[test]
    fn embedded_successful_handshake_drop_reaps_child() {
        let (command, pid_path) = welcomed_server("exec sleep 30");
        let link = EmbeddedLink::spawn_command(command, Duration::from_secs(5)).unwrap();
        assert!(link.latest_snapshot().is_none());
        assert!(!link.is_closed());
        drop(link);
        assert_startup_child_gone(&pid_path);
    }

    #[cfg(unix)]
    fn welcome_bytes() -> String {
        let welcome = crate::encode_welcome(&crate::Welcome {
            tick: 0,
            flight_time_s: 0.0,
        })
        .unwrap();
        // POSIX printf can emit an actual framed binary Welcome.
        welcome
            .iter()
            .map(|byte| format!("\\{:03o}", byte))
            .collect::<String>()
    }

    #[cfg(unix)]
    fn welcomed_server(after_welcome: &str) -> (std::process::Command, std::path::PathBuf) {
        injected_server(&format!("printf '{}'; {after_welcome}", welcome_bytes()))
    }

    #[cfg(unix)]
    fn wait_closed(link: &EmbeddedLink) {
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while !link.is_closed() {
            assert!(
                std::time::Instant::now() < deadline,
                "pipe failure did not close link"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(!link.send_input(&input(Vec::new())));
    }

    #[cfg(unix)]
    #[test]
    fn reader_eof_closes_input_while_child_is_still_alive() {
        let (command, pid_path) = welcomed_server("exec 1>&-; exec sleep 30");
        let mut link = EmbeddedLink::spawn_command(command, Duration::from_secs(5)).unwrap();
        wait_closed(&link);
        assert!(link.child.try_wait().unwrap().is_none());
        drop(link);
        assert_startup_child_gone(&pid_path);
    }

    #[cfg(unix)]
    #[test]
    fn writer_broken_pipe_closes_input_while_stdout_stays_open() {
        let hello_len = crate::encode_hello(&crate::Hello {
            client_name: "thessa-client-embedded".into(),
        })
        .unwrap()
        .len();
        // Consume Hello before Welcome so closing stdin cannot race startup.
        let (command, pid_path) = injected_server(&format!(
            "dd bs=1 count={hello_len} of=/dev/null 2>/dev/null; printf '{}'; exec 0<&-; exec sleep 30",
            welcome_bytes(),
        ));
        let mut link = EmbeddedLink::spawn_command(command, Duration::from_secs(5)).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while !link.is_closed() {
            assert!(
                std::time::Instant::now() < deadline,
                "broken writer did not close link"
            );
            link.send_input(&input(Vec::new()));
            std::thread::sleep(Duration::from_millis(1));
        }
        wait_closed(&link);
        assert!(link.child.try_wait().unwrap().is_none());
        drop(link);
        assert_startup_child_gone(&pid_path);
    }

    #[cfg(unix)]
    #[test]
    fn oversized_caller_packet_closes_link_without_queuing_it() {
        let (command, pid_path) = welcomed_server("exec sleep 30");
        let link = EmbeddedLink::spawn_command(command, Duration::from_secs(5)).unwrap();
        assert!(!link.send_input(&input(vec![
            Command::Stage;
            crate::MAX_COMMANDS_PER_INPUT + 1
        ])));
        assert!(link.is_closed());
        assert!(link.inputs.pending.lock().unwrap().is_none());
        drop(link);
        assert_startup_child_gone(&pid_path);
    }
}
