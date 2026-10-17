use thessa_render_wgpu::{Backend, BackendError, timing::GpuIntervals};

#[test]
fn ordinary_device_allocates_no_timestamp_profiler() {
    let backend = match Backend::bring_up() {
        Ok(b) => b,
        Err(BackendError::NoAdapter) => return,
        Err(e) => panic!("{e}"),
    };
    assert!(GpuIntervals::new(&backend, 5).is_none());
}

#[test]
fn profiled_intervals_report_real_timestamp_deltas_and_reuse_readback() {
    let backend = match Backend::bring_up_for_graphics_profiled(&Default::default()) {
        Ok(b) => b,
        Err(BackendError::NoAdapter) => return,
        Err(e) => panic!("{e}"),
    };
    let Some(timing) = GpuIntervals::new(&backend, 3) else {
        eprintln!("SKIP: timestamps unavailable");
        return;
    };
    let source = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: 4096,
        usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let destination = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: 4096,
        usage: wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    for _ in 0..3 {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        timing.mark(&mut encoder, 0);
        encoder.clear_buffer(&source, 0, None);
        timing.mark(&mut encoder, 1);
        encoder.copy_buffer_to_buffer(&source, 0, &destination, 0, 4096);
        timing.mark(&mut encoder, 2);
        timing.resolve(&mut encoder);
        backend.queue.submit([encoder.finish()]);
        let intervals = timing.read(&backend).unwrap();
        assert_eq!(intervals.len(), 2);
        assert!(
            intervals
                .iter()
                .all(|v| v.is_finite() && *v >= 0.0 && *v < 10.0)
        );
        assert!(
            intervals.iter().sum::<f64>() > 0.0,
            "hardware timestamps must advance"
        );
    }
}
