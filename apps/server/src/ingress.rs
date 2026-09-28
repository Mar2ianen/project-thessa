//! Bounded, transport-independent ingress into the authoritative sim driver.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

use super::*;

pub(super) const MAX_INGRESS_MESSAGES: usize = 512;
pub(super) const MAX_CLIENTS: usize = 256;
const MAX_PENDING_EDGE_COMMANDS: usize = 64;
const MAX_CONTINUOUS_SEGMENTS: usize = MAX_INGRESS_MESSAGES + 1;

/// Traffic from every transport into the sim driver. Continuous input is
/// kept in one latest-value slot per client. Edge-like input, guidance,
/// autopilot and subscriptions use a bounded async-safe queue. Leaves use a
/// separate id-set so cleanup cannot be lost when the event queue is full.
pub(super) enum Upstream {
    Input {
        id: String,
        input: ClientInput,
        sequence: u64,
    },
    Guidance {
        id: String,
        input: GuidanceInput,
        sequence: u64,
    },
    Autopilot {
        id: String,
        input: AutopilotInput,
        sequence: u64,
    },
    Subscribe {
        id: String,
        mailbox: Arc<OutboundMailbox>,
        sequence: u64,
    },
}

impl Upstream {
    pub(super) fn sequence(&self) -> u64 {
        match self {
            Self::Input { sequence, .. }
            | Self::Guidance { sequence, .. }
            | Self::Autopilot { sequence, .. }
            | Self::Subscribe { sequence, .. } => *sequence,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ConnectionState {
    Pending,
    Active,
    Closed,
}

/// Latest-value continuous-input slots, one bounded deque per client.
type ContinuousInputLog = Arc<Mutex<BTreeMap<String, VecDeque<(u64, ClientInput)>>>>;

#[derive(Clone)]
pub(super) struct IngressSender {
    events: tokio::sync::mpsc::Sender<Upstream>,
    latest_inputs: ContinuousInputLog,
    leaves: Arc<Mutex<BTreeSet<String>>>,
    pub(super) connections: Arc<Mutex<BTreeMap<String, ConnectionState>>>,
    next_connection: Arc<AtomicU64>,
    next_sequence: Arc<AtomicU64>,
    reliable_fences: Arc<Mutex<BTreeMap<String, u64>>>,
    ingress_lock: Arc<Mutex<()>>,
}

pub(super) struct IngressReceiver {
    events: tokio::sync::mpsc::Receiver<Upstream>,
    latest_inputs: ContinuousInputLog,
    leaves: Arc<Mutex<BTreeSet<String>>>,
    connections: Arc<Mutex<BTreeMap<String, ConnectionState>>>,
    reliable_fences: Arc<Mutex<BTreeMap<String, u64>>>,
    ingress_lock: Arc<Mutex<()>>,
}

impl IngressSender {
    pub(super) fn new() -> (Self, IngressReceiver) {
        let (events, event_receiver) = tokio::sync::mpsc::channel(MAX_INGRESS_MESSAGES);
        let latest_inputs = Arc::new(Mutex::new(BTreeMap::new()));
        let leaves = Arc::new(Mutex::new(BTreeSet::new()));
        let connections = Arc::new(Mutex::new(BTreeMap::new()));
        let next_sequence = Arc::new(AtomicU64::new(1));
        let reliable_fences = Arc::new(Mutex::new(BTreeMap::new()));
        let ingress_lock = Arc::new(Mutex::new(()));
        (
            Self {
                events,
                latest_inputs: latest_inputs.clone(),
                leaves: leaves.clone(),
                connections: connections.clone(),
                next_connection: Arc::new(AtomicU64::new(1)),
                next_sequence,
                reliable_fences: reliable_fences.clone(),
                ingress_lock: ingress_lock.clone(),
            },
            IngressReceiver {
                events: event_receiver,
                latest_inputs,
                leaves,
                connections,
                reliable_fences,
                ingress_lock,
            },
        )
    }

    /// Reserve an internal connection token before enqueueing Subscribe.
    /// This makes a queued Hello+EOF pair distinguishable from a later
    /// connection and bounds pending handshakes by the client limit.
    pub(super) fn reserve_connection(&self, label: &str) -> Option<String> {
        let token = self.next_connection.fetch_add(1, Ordering::Relaxed);
        let id = format!("{label}#{token}");
        self.reserve_exact_connection(id.clone()).then_some(id)
    }

    pub(super) fn reserve_exact_connection(&self, id: String) -> bool {
        let Ok(mut connections) = self.connections.lock() else {
            return false;
        };
        if connections.len() >= MAX_CLIENTS || connections.contains_key(&id) {
            return false;
        }
        connections.insert(id, ConnectionState::Pending);
        true
    }

    pub(super) fn can_enqueue(&self, id: &str) -> bool {
        self.connections
            .lock()
            .ok()
            .and_then(|connections| connections.get(id).copied())
            .is_some_and(|state| state != ConnectionState::Closed)
    }

    pub(super) fn mark_active_for_sender(&self, id: &str) -> bool {
        let Ok(mut connections) = self.connections.lock() else {
            return false;
        };
        let Some(state) = connections.get_mut(id) else {
            return false;
        };
        if *state != ConnectionState::Pending {
            return false;
        }
        *state = ConnectionState::Active;
        true
    }

    pub(super) fn send_input(&self, id: &str, input: ClientInput) -> bool {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return false;
        };
        if !self.can_enqueue(id) {
            return false;
        }
        let sequence = self.next_sequence.fetch_add(1, Ordering::Relaxed);
        if input.commands.iter().any(is_edge_command) {
            // Edge commands (Stage/Engine/Reset/ExecuteManeuver) are
            // reliable: silently dropping one while reporting success
            // desyncs client intent from authoritative state (e.g. a
            // staging event the client believes was delivered). On a full
            // shared event queue report backpressure (`false`) so the
            // caller tears the connection down loudly instead of
            // pretending the packet was handled. The driver drains every
            // iteration, so callers that observe `false` only on a truly
            // saturated queue; transient pressure surfaces as an explicit
            // disconnect/retry rather than a lost stage.
            match self.events.try_send(Upstream::Input {
                id: id.to_string(),
                input,
                sequence,
            }) {
                Ok(()) => {
                    if let Ok(mut fences) = self.reliable_fences.lock() {
                        fences.insert(id.to_string(), sequence);
                    }
                    true
                }
                Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                    eprintln!(
                        "[server] ingress full; rejecting edge input from {id} (backpressure, not delivered)"
                    );
                    false
                }
                Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => false,
            }
        } else {
            let Ok(mut latest) = self.latest_inputs.lock() else {
                return false;
            };
            if latest.len() >= MAX_CLIENTS && !latest.contains_key(id) {
                return false;
            }
            let segments = latest.entry(id.to_string()).or_default();
            let fence = self
                .reliable_fences
                .lock()
                .ok()
                .and_then(|fences| fences.get(id).copied())
                .unwrap_or(0);
            if segments.back().is_some_and(|(last, _)| *last > fence) {
                segments.back_mut().expect("segment exists").1 = input;
                true
            } else if segments.len() >= MAX_CONTINUOUS_SEGMENTS {
                false
            } else {
                segments.push_back((sequence, input));
                true
            }
        }
    }

    pub(super) fn send_guidance(&self, id: &str, input: GuidanceInput) -> bool {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return false;
        };
        if !self.can_enqueue(id) {
            return false;
        }
        let sequence = self.next_sequence.fetch_add(1, Ordering::Relaxed);
        match self.events.try_send(Upstream::Guidance {
            id: id.to_string(),
            input,
            sequence,
        }) {
            Ok(()) => {
                if let Ok(mut fences) = self.reliable_fences.lock() {
                    fences.insert(id.to_string(), sequence);
                }
                true
            }
            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                // Same reliability contract as edge inputs: never report
                // success for a dropped command. Backpressure as `false`.
                eprintln!(
                    "[server] ingress full; rejecting guidance from {id} (backpressure, not delivered)"
                );
                false
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => false,
        }
    }

    pub(super) fn send_autopilot(&self, id: &str, input: AutopilotInput) -> bool {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return false;
        };
        if !self.can_enqueue(id) {
            return false;
        }
        let sequence = self.next_sequence.fetch_add(1, Ordering::Relaxed);
        match self.events.try_send(Upstream::Autopilot {
            id: id.to_string(),
            input,
            sequence,
        }) {
            Ok(()) => {
                if let Ok(mut fences) = self.reliable_fences.lock() {
                    fences.insert(id.to_string(), sequence);
                }
                true
            }
            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                // Same reliability contract as edge inputs: never report
                // success for a dropped command. Backpressure as `false`.
                eprintln!(
                    "[server] ingress full; rejecting autopilot from {id} (backpressure, not delivered)"
                );
                false
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => false,
        }
    }

    pub(super) fn send_subscribe(&self, id: &str, mailbox: Arc<OutboundMailbox>) -> bool {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return false;
        };
        let Some(state) = self
            .connections
            .lock()
            .ok()
            .and_then(|connections| connections.get(id).copied())
        else {
            return false;
        };
        if state != ConnectionState::Pending {
            return false;
        }
        let sequence = self.next_sequence.fetch_add(1, Ordering::Relaxed);
        if self
            .events
            .try_send(Upstream::Subscribe {
                id: id.to_string(),
                mailbox,
                sequence,
            })
            .is_ok()
        {
            if let Ok(mut fences) = self.reliable_fences.lock() {
                fences.insert(id.to_string(), sequence);
            }
            true
        } else {
            self.close_connection(id);
            false
        }
    }

    pub(super) fn send_leave(&self, id: &str) -> bool {
        let Ok(mut connections) = self.connections.lock() else {
            return false;
        };
        let Some(state) = connections.get_mut(id) else {
            return false;
        };
        *state = ConnectionState::Closed;
        drop(connections);
        let Ok(mut leaves) = self.leaves.lock() else {
            return false;
        };
        leaves.insert(id.to_string());
        true
    }

    pub(super) fn close_connection(&self, id: &str) {
        if let Ok(mut connections) = self.connections.lock()
            && let Some(state) = connections.get_mut(id)
        {
            *state = ConnectionState::Closed;
        }
    }
}

impl IngressReceiver {
    pub(super) fn take_leaves(&self) -> BTreeSet<String> {
        let Ok(mut leaves) = self.leaves.lock() else {
            return BTreeSet::new();
        };
        std::mem::take(&mut *leaves)
    }

    pub(super) fn remove_latest_input(&self, id: &str) {
        if let Ok(mut latest) = self.latest_inputs.lock() {
            latest.remove(id);
        }
    }

    /// Atomically cut an ingress batch. Producers cannot allocate a sequence
    /// number and publish only half of a message while this boundary is held.
    /// Continuous segments after the last event in this slice stay queued for
    /// the next slice, so a later state packet cannot leap over an older
    /// reliable edge that was left behind by the per-iteration cap.
    pub(super) fn take_batch(
        &mut self,
        max_events: usize,
    ) -> (Vec<Upstream>, Vec<(String, u64, ClientInput)>, bool) {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return (Vec::new(), Vec::new(), false);
        };
        let mut events = Vec::with_capacity(max_events);
        while events.len() < max_events {
            match self.events.try_recv() {
                Ok(message) => events.push(message),
                Err(tokio::sync::mpsc::error::TryRecvError::Empty)
                | Err(tokio::sync::mpsc::error::TryRecvError::Disconnected) => break,
            }
        }
        // If the bounded event slice exhausted the queue, all currently
        // published continuous segments are safe to take. Otherwise the
        // remaining event(s) form an older fence for any later state.
        let watermark = (!self.events.is_empty())
            .then(|| events.iter().map(Upstream::sequence).max())
            .flatten();
        let mut continuous = Vec::new();
        if let Ok(mut latest) = self.latest_inputs.lock() {
            for (id, segments) in latest.iter_mut() {
                while let Some((sequence, _)) = segments.front()
                    && watermark.is_none_or(|limit| *sequence <= limit)
                {
                    let (sequence, input) = segments.pop_front().expect("segment exists");
                    continuous.push((id.clone(), sequence, input));
                }
            }
            latest.retain(|_, segments| !segments.is_empty());
        }
        let saturated = events.len() == max_events;
        (events, continuous, saturated)
    }

    pub(super) fn is_connection_live(&self, id: &str) -> bool {
        self.connections
            .lock()
            .ok()
            .and_then(|connections| connections.get(id).copied())
            .is_some_and(|state| state != ConnectionState::Closed)
    }

    pub(super) fn mark_active(&self, id: &str) -> bool {
        let Ok(mut connections) = self.connections.lock() else {
            return false;
        };
        let Some(state) = connections.get_mut(id) else {
            return false;
        };
        if *state != ConnectionState::Pending {
            return false;
        }
        *state = ConnectionState::Active;
        true
    }

    pub(super) fn close_connection(&self, id: &str) {
        if let Ok(mut connections) = self.connections.lock()
            && let Some(state) = connections.get_mut(id)
        {
            *state = ConnectionState::Closed;
        }
    }

    pub(super) fn finish_closed(&self, ids: &BTreeSet<String>) {
        if let Ok(mut connections) = self.connections.lock() {
            for id in ids {
                if connections.get(id) == Some(&ConnectionState::Closed) {
                    connections.remove(id);
                }
            }
        }
        if let Ok(mut fences) = self.reliable_fences.lock() {
            for id in ids {
                fences.remove(id);
            }
        }
    }
}

fn is_edge_command(command: &Command) -> bool {
    matches!(
        command,
        Command::Stage
            | Command::Engine { .. }
            | Command::Part { .. }
            | Command::Reset
            | Command::ExecuteManeuver { .. }
            | Command::ExecuteBurnPlan { .. }
    )
}

/// Mailbox for one client during a driver quantum. Continuous controls are
/// last-value-wins; event-like commands remain ordered. Warp and pause are
/// state votes, so an older vote can be replaced without losing a stage or
/// toggle event behind it.
#[derive(Default)]
pub(super) struct PendingInput {
    latest: Option<ClientInput>,
    commands: Vec<Command>,
    /// `engine_active` echo of the packet that contributed the newest edge
    /// command, with its sequence. Compared against the newest echo to tell
    /// a fresh user toggle apart from a stale pre-edge echo (see `take`).
    edge_echo: Option<bool>,
    edge_seq: u64,
    latest_seq: u64,
}

impl PendingInput {
    pub(super) fn push(&mut self, input: ClientInput, seq: u64) -> bool {
        let mut input = input;
        let commands = std::mem::take(&mut input.commands);
        let mut merged = std::mem::take(&mut self.commands);
        let mut edge_seen = false;
        for command in commands {
            match command {
                Command::SetWarp { .. } => {
                    merged.retain(|queued| !matches!(queued, Command::SetWarp { .. }));
                    merged.push(command);
                }
                Command::Pause { .. } => {
                    merged.retain(|queued| !matches!(queued, Command::Pause { .. }));
                    merged.push(command);
                }
                // Stage, part and explicit engine commands are edge/event-like:
                // every one must reach the authoritative state in order.
                event => {
                    edge_seen |= is_edge_command(&event);
                    merged.push(event);
                }
            }
        }
        if merged
            .iter()
            .filter(|command| is_edge_command(command))
            .count()
            > MAX_PENDING_EDGE_COMMANDS
        {
            self.commands = merged;
            return false;
        }
        // Pushes arrive in sequence order (the driver sorts the slice), so
        // the newest edge packet's echo is simply the last one observed.
        if edge_seen {
            self.edge_echo = Some(input.engine_active);
            self.edge_seq = seq;
        }
        self.latest = Some(input);
        self.latest_seq = seq;
        self.commands = merged;
        true
    }

    pub(super) fn take(&mut self) -> Option<ClientInput> {
        let mut input = self.latest.take()?;
        let mut commands = std::mem::take(&mut self.commands);
        // `apply_input` ignores the merged state's `engine_active` whenever
        // an edge is present, so a newer user toggle would be lost. But a
        // same-valued echo is stale/pre-edge and must NOT override the edge
        // result (Stage is a toggle: re-applying the old echo would undo it).
        // Append an explicit trailing edge only when the newest echo is both
        // strictly newer than the newest edge and different from the edge
        // packet's echo — proof the user toggled after the edge.
        if self.latest_seq > self.edge_seq
            && commands.iter().any(is_edge_command)
            && Some(input.engine_active) != self.edge_echo
        {
            commands.push(Command::Engine {
                active: input.engine_active,
            });
        }
        input.commands = commands;
        self.edge_echo = None;
        self.edge_seq = 0;
        self.latest_seq = 0;
        Some(input)
    }
}
