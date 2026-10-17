//! Explicit diagnostic timestamp intervals, not enabled in the normal path.
//! No timing model in simulation; callers adapt seconds to `thessa-perf`.
use crate::{Backend, BackendError};

pub(crate) fn required_features() -> wgpu::Features {
    wgpu::Features::TIMESTAMP_QUERY | wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS
}

/// One reusable query/readback allocation for serial headless diagnostics.
/// The caller must submit and complete a frame before reading/reusing it.
pub struct GpuIntervals {
    query: wgpu::QuerySet,
    resolved: wgpu::Buffer,
    readback: wgpu::Buffer,
    count: u32,
    seconds_per_tick: f64,
}

impl GpuIntervals {
    /// `None` is a supported, explicitly unavailable capability fallback.
    pub fn new(backend: &Backend, count: u32) -> Option<Self> {
        if !(2..=64).contains(&count) || !backend.device.features().contains(required_features()) {
            return None;
        }
        let query = backend.device.create_query_set(&wgpu::QuerySetDescriptor {
            label: Some("headless-perf-intervals"),
            ty: wgpu::QueryType::Timestamp,
            count,
        });
        let buffer = |usage| {
            backend.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("headless-perf-intervals"),
                size: u64::from(count) * 8,
                usage,
                mapped_at_creation: false,
            })
        };
        Some(Self {
            query,
            resolved: buffer(wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC),
            readback: buffer(wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST),
            count,
            seconds_per_tick: f64::from(backend.queue.get_timestamp_period()) * 1e-9,
        })
    }

    pub fn mark(&self, encoder: &mut wgpu::CommandEncoder, index: u32) {
        assert!(index < self.count);
        encoder.write_timestamp(&self.query, index);
    }

    pub fn resolve(&self, encoder: &mut wgpu::CommandEncoder) {
        encoder.resolve_query_set(&self.query, 0..self.count, &self.resolved, 0);
        encoder.copy_buffer_to_buffer(
            &self.resolved,
            0,
            &self.readback,
            0,
            u64::from(self.count) * 8,
        );
    }

    /// Read consecutive interval lengths in seconds, not host submit/wait time.
    /// Mapping is intentionally outside the frame wall-time sample; this API
    /// is for serial diagnostics, not a non-blocking window frame profiler.
    pub fn read(&self, backend: &Backend) -> Result<Vec<f64>, BackendError> {
        let (tx, rx) = std::sync::mpsc::channel();
        self.readback
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |r| {
                let _ = tx.send(r);
            });
        backend
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| fail(e.to_string()))?;
        rx.recv()
            .map_err(|e| fail(e.to_string()))?
            .map_err(|e| fail(e.to_string()))?;
        let data = self
            .readback
            .slice(..)
            .get_mapped_range()
            .map_err(|e| fail(e.to_string()))?;
        let ticks: Vec<_> = data
            .as_chunks::<8>()
            .0
            .iter()
            .map(|v| u64::from_le_bytes(*v))
            .collect();
        let intervals = decode(&ticks, self.seconds_per_tick);
        drop(data);
        self.readback.unmap();
        intervals.ok_or_else(|| fail("invalid GPU timestamp sequence".into()))
    }
}

fn fail(s: String) -> BackendError {
    BackendError::Device(s)
}

fn decode(ticks: &[u64], period: f64) -> Option<Vec<f64>> {
    if ticks.len() < 2 || !period.is_finite() || period <= 0.0 {
        return None;
    }
    ticks
        .windows(2)
        .map(|pair| {
            pair[1]
                .checked_sub(pair[0])
                .map(|ticks| ticks as f64 * period)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn timestamp_deltas_are_seconds_and_invalid_sequences_are_unavailable() {
        let intervals = decode(&[100, 200, 450], 0.5e-9).unwrap();
        for (actual, expected) in intervals.iter().zip([50e-9, 125e-9]) {
            assert!((actual - expected).abs() < 1e-20);
        }
        assert_eq!(decode(&[10, 9], 1e-9), None);
        assert_eq!(decode(&[0, 1], f64::NAN), None);
    }
}
