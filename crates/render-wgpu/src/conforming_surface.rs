//! Experimental persistent-bisector surface consumer. Borrows the existing
//! terrain page publication; no second height allocator, streamer or grid LOD.
use crate::{Backend, BackendError};
use thessa_rcbt_core::{
    conforming_surface::precision_charts,
    material_cache::{SlotEntry, material_directory_with_capacity},
    shaders,
};
use wgpu::util::DeviceExt;

/// Live resource identity, not a frozen snapshot. Existing terrain publication
/// owns uploads and slot retirement. Encode after publication, before reuse;
/// clones retain resources but do not pin old page contents or revisions.
#[derive(Debug, Clone)]
pub struct HeightSurfaceSource {
    directory: wgpu::Buffer,
    info: wgpu::Buffer,
    metadata: wgpu::Buffer,
    residuals: wgpu::Buffer,
    radius_m: f64,
}
impl HeightSurfaceSource {
    pub(crate) fn new(
        backend: &Backend,
        metadata: &wgpu::Buffer,
        residuals: &wgpu::Buffer,
        radius_m: f64,
        ids: &[u64],
    ) -> Result<Self, BackendError> {
        let data = directory(ids)?;
        Ok(Self {
            info: backend
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("height-directory-extent"),
                    contents: &directory_extent(ids).to_le_bytes(),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                }),
            directory: backend
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("canonical-height-directory"),
                    contents: &data,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                }),
            metadata: metadata.clone(),
            residuals: residuals.clone(),
            radius_m,
        })
    }
    pub(crate) fn publish(&self, backend: &Backend, ids: &[u64]) -> Result<(), BackendError> {
        backend
            .queue
            .write_buffer(&self.directory, 0, &directory(ids)?);
        backend
            .queue
            .write_buffer(&self.info, 0, &directory_extent(ids).to_le_bytes());
        Ok(())
    }
}
fn directory_extent(ids: &[u64]) -> u32 {
    ids.iter()
        .filter(|&&id| id >= 8)
        .map(|&id| (63 - id.leading_zeros() - 3) / 2)
        .max()
        .unwrap_or(0)
}
fn directory(ids: &[u64]) -> Result<Vec<u8>, BackendError> {
    let mut entries = Vec::new();
    for (slot, &node_id) in ids.iter().enumerate() {
        if node_id == 0 {
            continue;
        }
        entries.push(SlotEntry {
            slot: slot as u32,
            node_id,
            generation: 0,
        });
    }
    let mut unique = std::collections::BTreeSet::new();
    if entries.iter().any(|e| !unique.insert(e.node_id)) {
        return Err(BackendError::Device(
            "duplicate canonical surface page".into(),
        ));
    }
    let table = material_directory_with_capacity(entries, ids.len())
        .ok_or_else(|| BackendError::Device("height directory overflow".into()))?;
    Ok(table
        .into_iter()
        .flatten()
        .flat_map(u32::to_le_bytes)
        .collect())
}

pub const VERTEX_BYTES: u64 = 48;
/// Per allocation slot: three 48-byte vertices (position+height, normal+face,
/// exact UV numerators+source slot+level). No culled or regular-grid geometry.
pub struct ConformingSurface {
    pub vertices: wgpu::Buffer,
    charts: wgpu::Buffer,
    pipeline: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    capacity: u32,
    radius_m: f64,
}
impl ConformingSurface {
    pub fn new(
        backend: &Backend,
        pool: &wgpu::Buffer,
        source: &HeightSurfaceSource,
    ) -> Result<Self, BackendError> {
        if !pool.size().is_multiple_of(32) || pool.size() == 0 {
            return Err(BackendError::Device("invalid bisector pool stride".into()));
        }
        let capacity = u32::try_from(pool.size() / 32)
            .map_err(|_| BackendError::Device("bisector capacity overflow".into()))?;
        let size = u64::from(capacity) * 3 * VERTEX_BYTES;
        if size > backend.device.limits().max_storage_buffer_binding_size {
            return Err(BackendError::Device(
                "surface vertex capacity exceeds storage limit".into(),
            ));
        }
        let vertices = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("persistent-cbt-surface"),
            size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let charts = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("surface-precision-charts"),
            size: 6 * 128,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let module = backend
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("canonical-cbt-surface"),
                source: wgpu::ShaderSource::Wgsl(shader_source().into()),
            });
        let pipeline = backend
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("emit-canonical-cbt-surface"),
                layout: None,
                module: &module,
                entry_point: Some("emit_surface"),
                compilation_options: Default::default(),
                cache: None,
            });
        let bind = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: None,
                layout: &pipeline.get_bind_group_layout(0),
                entries: &[
                    pool,
                    &source.directory,
                    &source.metadata,
                    &source.residuals,
                    &charts,
                    &vertices,
                    &source.info,
                ]
                .into_iter()
                .enumerate()
                .map(|(binding, buffer)| wgpu::BindGroupEntry {
                    binding: binding as u32,
                    resource: buffer.as_entire_binding(),
                })
                .collect::<Vec<_>>(),
            });
        let surface = Self {
            vertices,
            charts,
            pipeline,
            bind,
            capacity,
            radius_m: source.radius_m,
        };
        surface.update_camera(backend, [0.0; 3])?;
        Ok(surface)
    }
    /// Body-fixed camera metres, subtracted in f64 before any GPU upload.
    pub fn update_camera(&self, backend: &Backend, camera: [f64; 3]) -> Result<(), BackendError> {
        let charts = precision_charts(self.radius_m, camera)
            .ok_or_else(|| BackendError::Device("invalid surface camera/radius".into()))?;
        let mut data = Vec::with_capacity(768);
        for (tile, frame) in charts {
            data.extend(
                frame
                    .words()
                    .into_iter()
                    .flatten()
                    .flat_map(f32::to_le_bytes),
            );
            data.extend(
                [u32::from(tile.face), u32::from(tile.level), tile.x, tile.y]
                    .into_iter()
                    .flat_map(u32::to_le_bytes),
            );
        }
        backend.queue.write_buffer(&self.charts, 0, &data);
        Ok(())
    }
    /// Must follow topology publication. Page uploads and camera updates must
    /// precede submission; readbacks are diagnostic only.
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder) {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind, &[]);
        pass.dispatch_workgroups(self.capacity.div_ceil(64), 1, 1);
    }
    pub fn storage_bytes(&self) -> u64 {
        self.vertices.size() + self.charts.size()
    }
    /// Allocation slots covered by the emitted vertex stream (three vertices
    /// each). Diagnostic caster consumers draw the full slot range.
    pub fn slot_count(&self) -> u32 {
        self.capacity
    }
}
fn shader_source() -> String {
    format!(
        "{}\nvar<private> p:Params;\n{}\n{}\n{}\n{}\n{}",
        thessa_rcbt_wgpu::sphere::GEOMETRY_WGSL,
        shaders::PRECISION_WGSL,
        shaders::TILE_FRAME_WGSL,
        shaders::SURFACE_SAMPLE_WGSL,
        shaders::PAGE_DIRECTORY_WGSL,
        include_str!("conforming_surface.wgsl")
    )
}

#[cfg(test)]
mod tests {
    #[test]
    fn surface_shader_requires_no_native_capabilities() {
        let source = super::shader_source();
        let module = naga::front::wgsl::parse_str(&source)
            .unwrap_or_else(|e| panic!("{}", e.emit_to_string(&source)));
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::empty(),
        )
        .validate(&module)
        .unwrap();
    }
}
