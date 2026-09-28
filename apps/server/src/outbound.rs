//! Per-client outbound mailboxes shared by the stdio and TCP transports.

use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};

pub(super) const RELIABLE_OUTBOUND_CAPACITY: usize = 32;

struct OutboundState {
    reliable: VecDeque<Vec<u8>>,
    latest_snapshot: Option<Arc<Vec<u8>>>,
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

    pub(super) fn try_next(&self) -> Option<OutboundFrame> {
        let mut state = self.state.lock().ok()?;
        state
            .reliable
            .pop_front()
            .map(OutboundFrame::Owned)
            .or_else(|| state.latest_snapshot.take().map(OutboundFrame::Shared))
    }

    pub(super) fn blocking_next(&self) -> Option<OutboundFrame> {
        let mut state = self.state.lock().ok()?;
        loop {
            if let Some(frame) = state
                .reliable
                .pop_front()
                .map(OutboundFrame::Owned)
                .or_else(|| state.latest_snapshot.take().map(OutboundFrame::Shared))
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
