//! Per-client outbound mailboxes shared by the stdio and TCP transports.

use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};

pub(super) const RELIABLE_OUTBOUND_CAPACITY: usize = 32;

struct OutboundState {
    reliable: VecDeque<Vec<u8>>,
    latest_snapshot: Option<Arc<Vec<u8>>>,
    latest_fleet_snapshot: Option<Arc<Vec<u8>>>,
    latest_fleet_geometry: Option<Arc<Vec<u8>>>,
    geometry_revision: u64,
    closed: bool,
}

#[derive(Debug)]
pub(super) enum OutboundFrame {
    Owned(Vec<u8>),
    Shared(Arc<Vec<u8>>),
}

impl AsRef<[u8]> for OutboundFrame {
    fn as_ref(&self) -> &[u8] {
        match self {
            Self::Owned(frame) => frame,
            Self::Shared(frame) => frame.as_slice(),
        }
    }
}

/// Per-client outbound mailbox. Welcome and future control frames are
/// bounded reliable messages; snapshots are a single latest-wins slot. The
/// simulation thread only takes a short mutex and never waits for a socket.
pub(super) struct OutboundMailbox {
    state: Mutex<OutboundState>,
    blocking_wake: Condvar,
    async_wake: tokio::sync::Notify,
}

impl OutboundMailbox {
    pub(super) fn new() -> Self {
        Self {
            state: Mutex::new(OutboundState {
                reliable: VecDeque::with_capacity(RELIABLE_OUTBOUND_CAPACITY),
                latest_snapshot: None,
                latest_fleet_snapshot: None,
                latest_fleet_geometry: None,
                geometry_revision: 0,
                closed: false,
            }),
            blocking_wake: Condvar::new(),
            async_wake: tokio::sync::Notify::new(),
        }
    }

    pub(super) fn push_reliable(&self, frame: Vec<u8>) -> bool {
        let Ok(mut state) = self.state.lock() else {
            return false;
        };
        if state.closed || state.reliable.len() >= RELIABLE_OUTBOUND_CAPACITY {
            return false;
        }
        state.reliable.push_back(frame);
        drop(state);
        self.blocking_wake.notify_one();
        self.async_wake.notify_one();
        true
    }

    pub(super) fn replace_snapshot(&self, frame: Arc<Vec<u8>>) -> bool {
        let Ok(mut state) = self.state.lock() else {
            return false;
        };
        if state.closed {
            return false;
        }
        state.latest_snapshot = Some(frame);
        drop(state);
        self.blocking_wake.notify_one();
        self.async_wake.notify_one();
        true
    }

    /// Latest-wins fleet snapshot slot, drained right after the primary
    /// snapshot. Includes explicit empty membership in single-vehicle operation.
    pub(super) fn replace_fleet_snapshot(&self, frame: Arc<Vec<u8>>) -> bool {
        let Ok(mut state) = self.state.lock() else {
            return false;
        };
        if state.closed {
            return false;
        }
        state.latest_fleet_snapshot = Some(frame);
        drop(state);
        self.blocking_wake.notify_one();
        self.async_wake.notify_one();
        true
    }

    /// Persistent revision marker suppresses retransmission after draining.
    /// A new mailbox always receives the current catalogue. Coalescing may
    /// skip revisions; consumers match the snapshot's revision, never guess.
    pub(super) fn replace_fleet_geometry(&self, revision: u64, frame: Arc<Vec<u8>>) -> bool {
        let Ok(mut state) = self.state.lock() else {
            return false;
        };
        if state.closed || revision < state.geometry_revision || revision == 0 {
            return false;
        }
        if revision == state.geometry_revision {
            return true;
        }
        state.latest_fleet_geometry = Some(frame);
        state.geometry_revision = revision;
        drop(state);
        self.blocking_wake.notify_one();
        self.async_wake.notify_one();
        true
    }

    fn drain_snapshots(state: &mut OutboundState) -> Option<OutboundFrame> {
        state
            .latest_snapshot
            .take()
            .map(OutboundFrame::Shared)
            .or_else(|| {
                state
                    .latest_fleet_geometry
                    .take()
                    .map(OutboundFrame::Shared)
            })
            .or_else(|| {
                state
                    .latest_fleet_snapshot
                    .take()
                    .map(OutboundFrame::Shared)
            })
    }

    pub(super) fn try_next(&self) -> Option<OutboundFrame> {
        let mut state = self.state.lock().ok()?;
        state
            .reliable
            .pop_front()
            .map(OutboundFrame::Owned)
            .or_else(|| Self::drain_snapshots(&mut state))
    }

    pub(super) fn blocking_next(&self) -> Option<OutboundFrame> {
        let mut state = self.state.lock().ok()?;
        loop {
            if let Some(frame) = state
                .reliable
                .pop_front()
                .map(OutboundFrame::Owned)
                .or_else(|| Self::drain_snapshots(&mut state))
            {
                return Some(frame);
            }
            if state.closed {
                return None;
            }
            state = match self.blocking_wake.wait(state) {
                Ok(guard) => guard,
                // A poisoned waiter cannot proceed; ending the writer
                // thread is strictly better than panicking the server.
                Err(_) => return None,
            };
        }
    }

    pub(super) async fn next(&self) -> Option<OutboundFrame> {
        loop {
            let notified = self.async_wake.notified();
            if let Some(frame) = self.try_next() {
                return Some(frame);
            }
            if self.state.lock().ok()?.closed {
                return None;
            }
            notified.await;
        }
    }

    pub(super) fn close(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.closed = true;
        }
        self.blocking_wake.notify_all();
        self.async_wake.notify_waiters();
    }
}
