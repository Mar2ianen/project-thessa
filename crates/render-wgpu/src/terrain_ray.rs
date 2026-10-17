//! Ray membership of the existing generated terrain pool. No mesh expansion,
//! readback, camera culling, or duplicate vertex storage.
use crate::{Backend, BackendError, terrain::VERTICES_PER_PATCH};
use thessa_rcbt_core::tile_anchor::TileAnchor;
use wgpu::util::DeviceExt;

pub(crate) const TERRAIN_TAG: u32 = 1 << 31;
pub(crate) const INDEX_COUNT: u32 = 32 * 32 * 6;
pub(crate) const INDEX_BYTES: u64 = INDEX_COUNT as u64 * 4;
/// Conservative planning charge includes the borrowed vertex slot, even though
/// it is not allocated a second time. Driver-owned AS memory is not measured.
pub(crate) const SLOT_BYTES: u64 = VERTICES_PER_PATCH as u64 * 32 + 256 + 128;

/// Retained resource identity. Cloning shares GPU resources, not geometry.
/// Obtain from `TerrainRenderer::ray_source`; publication is synchronized via
/// `MeshRenderer::sync_rt_terrain` after height/membership updates, before draw.
#[derive(Debug, Clone)]
pub struct TerrainRaySource {
    pub(crate) ocean: Option<crate::ocean::OceanBinding>,
    pub(crate) vertices: wgpu::Buffer,
    pub(crate) directory: wgpu::Buffer,
    pub(crate) material: wgpu::TextureView,
    pub(crate) sampler: wgpu::Sampler,
    pub(crate) radius_m: f64,
    pub(crate) capacity: usize,
}

pub(crate) struct TerrainRays {
    pub source: TerrainRaySource,
    pub indices: wgpu::Buffer,
    pub size: wgpu::BlasTriangleGeometrySizeDescriptor,
    pub slots: Vec<Option<wgpu::Blas>>,
    revisions: Vec<u64>,
    dirty_slots: Vec<bool>,
    pub dirty: Vec<usize>,
    pub active: Vec<(usize, TileAnchor)>,
    pub capacity: usize,
    pub reason: Option<String>,
}

impl TerrainRays {
    pub fn new(device: &wgpu::Device, source: TerrainRaySource, capacity: usize) -> Self {
        let indices = (0..32u32).flat_map(|y| {
            (0..32u32).flat_map(move |x| {
                let a = y * 33 + x;
                [a, a + 1, a + 33, a + 1, a + 34, a + 33]
            })
        });
        Self {
            indices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("terrain-rt-grid"),
                contents: &indices.flat_map(u32::to_le_bytes).collect::<Vec<_>>(),
                usage: wgpu::BufferUsages::BLAS_INPUT,
            }),
            size: wgpu::BlasTriangleGeometrySizeDescriptor {
                vertex_format: wgpu::VertexFormat::Float32x3,
                vertex_count: VERTICES_PER_PATCH,
                index_format: Some(wgpu::IndexFormat::Uint32),
                index_count: Some(INDEX_COUNT),
                flags: wgpu::AccelerationStructureGeometryFlags::OPAQUE,
            },
            slots: vec![None; source.capacity],
            revisions: vec![0; source.capacity],
            dirty_slots: vec![false; source.capacity],
            dirty: Vec::new(),
            active: Vec::new(),
            source,
            capacity,
            reason: None,
        }
    }

    pub fn sync(
        &mut self,
        backend: &Backend,
        terrain: &crate::terrain::TerrainRenderer,
    ) -> Result<(), BackendError> {
        let (vertices, ids, revisions, anchors) = terrain.ray_publication();
        if vertices != &self.source.vertices {
            return Err(BackendError::Device(
                "RT terrain source identity changed; recreate the mesh consumer".into(),
            ));
        }
        self.active.clear();
        let count = ids.iter().filter(|&&id| id != 0).count();
        if count > self.capacity {
            // Never leave stale/partial blockers. Retire the whole terrain
            // contribution; raster shadows and environment misses remain.
            self.slots.fill(None);
            self.revisions.fill(0);
            self.dirty.clear();
            self.dirty_slots.fill(false);
            self.reason = Some(format!(
                "RT terrain partition has {count} patches, budget admits {}; raster terrain fallback",
                self.capacity
            ));
            return Ok(());
        }
        self.reason = None;
        for (slot, &id) in ids.iter().enumerate() {
            if id == 0 {
                self.slots[slot] = None;
                self.revisions[slot] = 0;
                self.dirty_slots[slot] = false;
                continue;
            }
            if self.slots[slot].is_none() {
                self.slots[slot] = Some(backend.device.create_blas(
                    &wgpu::CreateBlasDescriptor {
                        label: Some("terrain-slot-blas"),
                        flags: wgpu::AccelerationStructureFlags::PREFER_FAST_TRACE,
                        update_mode: wgpu::AccelerationStructureUpdateMode::Build,
                    },
                    wgpu::BlasGeometrySizeDescriptors::Triangles {
                        descriptors: vec![self.size.clone()],
                    },
                ));
            }
            if self.revisions[slot] != revisions[slot] {
                self.dirty_slots[slot] = true;
            }
            self.revisions[slot] = revisions[slot];
            self.active.push((slot, anchors[slot]));
        }
        self.dirty.clear();
        self.dirty.extend(
            self.dirty_slots
                .iter()
                .enumerate()
                .filter_map(|(slot, &dirty)| dirty.then_some(slot)),
        );
        Ok(())
    }

    pub fn mark_built(&mut self) {
        for slot in self.dirty.drain(..) {
            self.dirty_slots[slot] = false;
        }
    }

    pub fn builds(&self) -> Vec<wgpu::BlasBuildEntry<'_>> {
        self.dirty
            .iter()
            .map(|&slot| wgpu::BlasBuildEntry {
                blas: self.slots[slot]
                    .as_ref()
                    .expect("dirty active terrain BLAS"),
                geometry: wgpu::BlasGeometries::TriangleGeometries(vec![
                    wgpu::BlasTriangleGeometry {
                        size: &self.size,
                        vertex_buffer: &self.source.vertices,
                        first_vertex: slot as u32 * VERTICES_PER_PATCH,
                        vertex_stride: 32,
                        index_buffer: Some(&self.indices),
                        first_index: Some(0),
                        transform_buffer: None,
                        transform_buffer_offset: None,
                    },
                ]),
            })
            .collect()
    }
}
