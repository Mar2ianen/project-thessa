//! Fixed-capacity rolling storage for frame records.

use std::collections::VecDeque;

/// Fixed-capacity ring buffer. Push overwrites the oldest element once full,
/// so capture writing never grows memory unboundedly during a session.
#[derive(Debug, Clone)]
pub struct RingBuffer<T> {
    inner: VecDeque<T>,
    capacity: usize,
}

impl<T> RingBuffer<T> {
    pub fn new(capacity: usize) -> Self {
        Self {
            inner: VecDeque::with_capacity(capacity),
            capacity: capacity.max(1),
        }
    }

    pub fn push(&mut self, value: T) {
        if self.inner.len() >= self.capacity {
            self.inner.pop_front();
        }
        self.inner.push_back(value);
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.inner.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overwrites_oldest_when_full() {
        let mut ring = RingBuffer::new(2);
        ring.push(1);
        ring.push(2);
        ring.push(3);
        let collected: Vec<_> = ring.iter().copied().collect();
        assert_eq!(collected, vec![2, 3]);
    }
}
