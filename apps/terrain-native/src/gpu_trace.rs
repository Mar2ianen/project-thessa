//! Opt-in, crash-durable headless stage tracing. Serial fences deliberately
//! change scheduling; this is not the ordinary performance-capture path.
use std::{error::Error, fs::File, io::Write, path::Path, time::Instant};
use thessa_render_wgpu::{Backend, mesh::MeshDrawStats};

pub const DEFAULT_MESH_TRIANGLE_LIMIT: u64 = 1_000_000;

pub struct GpuTrace {
    file: File,
    started: Instant,
    mesh_triangle_limit: u64,
}

impl GpuTrace {
    pub fn create(path: &Path, mesh_triangle_limit: u64) -> Result<Self, Box<dyn Error>> {
        if mesh_triangle_limit == 0 {
            return Err("GPU trace mesh triangle limit must be positive".into());
        }
        let mut file = File::options().write(true).create_new(true).open(path)?;
        writeln!(
            file,
            "version = 1\nmode = 'serial-gpu-stage-diagnostic'\nprocess_id = {}\nmesh_triangle_limit = {mesh_triangle_limit}",
            std::process::id()
        )?;
        let mut trace = Self {
            file,
            started: Instant::now(),
            mesh_triangle_limit,
        };
        trace.event(
            0,
            "startup",
            "created",
            "GPU trace enabled; scheduling differs from ordinary rendering",
        )?;
        Ok(trace)
    }

    /// Flush before submission so a context loss cannot erase the last intent.
    pub fn event(
        &mut self,
        frame: u64,
        stage: &str,
        state: &str,
        detail: &str,
    ) -> Result<(), Box<dyn Error>> {
        let fields = std::collections::BTreeMap::from([
            ("frame", frame.to_string()),
            ("stage", stage.to_string()),
            ("state", state.to_string()),
            (
                "elapsed_ms",
                (self.started.elapsed().as_secs_f64() * 1000.0).to_string(),
            ),
            ("detail", detail.to_string()),
        ]);
        writeln!(self.file, "\n[[event]]\n{}", toml::to_string(&fields)?)?;
        self.file.flush()?;
        self.file.sync_data()?;
        Ok(())
    }

    /// Admission rejects work; it never changes density, geometry or effects.
    /// Triangle counts are not a GPU-duration bound or protection from bugs.
    pub fn admit_mesh(
        &mut self,
        frame: u64,
        stage: &str,
        triangles: u64,
    ) -> Result<(), Box<dyn Error>> {
        if triangles > self.mesh_triangle_limit {
            let reason = format!(
                "{triangles} submitted mesh triangles exceed trace limit {}; no stage submission",
                self.mesh_triangle_limit
            );
            self.event(frame, stage, "admission_rejected", &reason)?;
            return Err(reason.into());
        }
        Ok(())
    }

    pub fn workload(
        &mut self,
        frame: u64,
        mesh: Option<MeshDrawStats>,
        shadows: usize,
    ) -> Result<(), Box<dyn Error>> {
        self.event(frame,"frame","workload",&format!("primary={mesh:?}; potential_shadow_layers={shadows}; exact shadow counts follow encoding"))?;
        self.admit_mesh(frame, "primary", mesh.map_or(0, |s| s.triangles))
    }

    pub fn checkpoint(
        &mut self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        frame: u64,
        stage: &str,
        triangles: u64,
    ) -> Result<(), Box<dyn Error>> {
        self.admit_mesh(frame, stage, triangles)?;
        encoder.insert_debug_marker(stage);
        let next = backend
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("native-gpu-trace-stage"),
            });
        let finished = std::mem::replace(encoder, next).finish();
        self.submit(backend, [finished], frame, stage)
    }

    pub fn submit(
        &mut self,
        backend: &Backend,
        commands: impl IntoIterator<Item = wgpu::CommandBuffer>,
        frame: u64,
        stage: &str,
    ) -> Result<(), Box<dyn Error>> {
        self.event(
            frame,
            stage,
            "submit_intent",
            "written before queue.submit; not proof of GPU execution",
        )?;
        backend.queue.submit(commands);
        match backend.device.poll(wgpu::PollType::wait_indefinitely()) {
            Ok(_) => self.event(frame, stage, "gpu_completed", "queue fence completed"),
            Err(error) => {
                self.event(frame, stage, "poll_failed", &error.to_string())?;
                Err(error.into())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejected_work_is_durable_and_existing_evidence_is_never_overwritten() {
        let path =
            std::env::temp_dir().join(format!("thessa-gpu-trace-{}.toml", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let mut trace = GpuTrace::create(&path, 100).unwrap();
        trace.admit_mesh(0, "primary", 100).unwrap();
        assert!(trace.admit_mesh(0, "shadows", 101).is_err());
        assert!(
            trace
                .workload(
                    0,
                    Some(MeshDrawStats {
                        instances: 17992,
                        candidate_instances: 17992,
                        candidate_triangles: 148572518,
                        draw_calls: 37,
                        draw_commands: 37,
                        triangles: 148572518,
                        frame_upload_bytes: 1152064,
                    }),
                    12
                )
                .is_err(),
            "the WORLD142 workload must be rejected without a backend"
        );
        // Inspect while the writer is live: failures must not depend on Drop.
        let text = std::fs::read_to_string(&path).unwrap();
        let doc: toml::Value = toml::from_str(&text).unwrap();
        assert_eq!(
            doc["process_id"].as_integer(),
            Some(i64::from(std::process::id()))
        );
        let events = doc["event"].as_array().unwrap();
        assert_eq!(
            events.last().unwrap()["state"].as_str(),
            Some("admission_rejected")
        );
        assert_eq!(events.last().unwrap()["stage"].as_str(), Some("primary"));
        assert!(
            events
                .iter()
                .all(|e| e["state"].as_str() != Some("submit_intent"))
        );
        assert!(GpuTrace::create(&path, 100).is_err());
        assert_eq!(std::fs::read_to_string(&path).unwrap(), text);
        drop(trace);
        std::fs::remove_file(path).unwrap();
    }
}
