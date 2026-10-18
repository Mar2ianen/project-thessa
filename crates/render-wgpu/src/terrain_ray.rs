//! Ray membership of the existing generated terrain pool. No mesh expansion,
//! readback, camera culling, or duplicate vertex storage.
use crate::{Backend, BackendError, terrain::VERTICES_PER_PATCH};
use std::sync::{Arc, Mutex};
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
/// `ConformingSurface::ray_source` instead follows its shared encoded epoch
/// automatically at RayScene encoding; do not call the legacy patch sync API.
#[derive(Debug, Clone)]
pub struct TerrainRaySource {
    pub(crate) conforming: Option<Arc<Mutex<crate::conforming_surface::SurfacePublicationState>>>,
    pub(crate) conforming_configuration: u64,
    pub(crate) ocean: Option<crate::ocean::OceanBinding>,
    pub(crate) vertices: wgpu::Buffer,
    pub(crate) directory: wgpu::Buffer,
    pub(crate) material: wgpu::TextureView,
    pub(crate) sampler: wgpu::Sampler,
    pub(crate) radius_m: f64,
    pub(crate) capacity: usize,
}

impl TerrainRaySource {
    pub(crate) fn same_resources(&self, other: &Self) -> bool {
        self.vertices == other.vertices
            && self.directory == other.directory
            && self.material == other.material
            && self.sampler == other.sampler
            && self.conforming_configuration == other.conforming_configuration
            && match (&self.conforming, &other.conforming) {
                (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                (None, None) => true,
                _ => false,
            }
            && match (&self.ocean, &other.ocean) {
                (Some(a), Some(b)) => {
                    a.frame == b.frame && a.field == b.field && a.sampler == b.sampler
                }
                (None, None) => true,
                _ => false,
            }
    }
    pub(crate) fn index_bytes(&self) -> u64 {
        if self.conforming.is_some() {
            0
        } else {
            INDEX_BYTES
        }
    }
    pub(crate) fn slot_bytes(&self) -> u64 {
        if self.conforming.is_some() {
            2048 * 3 * crate::conforming_surface::VERTEX_BYTES + 384
        } else {
            SLOT_BYTES
        }
    }
    fn size(&self, slot: usize) -> wgpu::BlasTriangleGeometrySizeDescriptor {
        let conforming = self.conforming.is_some();
        let triangles = (self.vertices.size() / (3 * crate::conforming_surface::VERTEX_BYTES))
            .saturating_sub(slot as u64 * 2048)
            .min(2048) as u32;
        wgpu::BlasTriangleGeometrySizeDescriptor {
            vertex_format: wgpu::VertexFormat::Float32x3,
            vertex_count: if conforming {
                triangles * 3
            } else {
                VERTICES_PER_PATCH
            },
            index_format: (!conforming).then_some(wgpu::IndexFormat::Uint32),
            index_count: (!conforming).then_some(INDEX_COUNT),
            flags: wgpu::AccelerationStructureGeometryFlags::OPAQUE,
        }
    }
}

pub(crate) struct TerrainRayPlacement {
    pub origin_m: [f64; 3],
    pub radial: [f32; 3],
}

pub(crate) struct TerrainRays {
    pub source: TerrainRaySource,
    pub indices: Option<wgpu::Buffer>,
    sizes: Vec<wgpu::BlasTriangleGeometrySizeDescriptor>,
    pub slots: Vec<Option<wgpu::Blas>>,
    revisions: Vec<u64>,
    dirty_slots: Vec<bool>,
    pub dirty: Vec<usize>,
    pub active: Vec<(usize, TerrainRayPlacement)>,
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
        let mut result = Self {
            indices: source.conforming.is_none().then(|| {
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("terrain-rt-grid"),
                    contents: &indices.flat_map(u32::to_le_bytes).collect::<Vec<_>>(),
                    usage: wgpu::BufferUsages::BLAS_INPUT,
                })
            }),
            sizes: (0..source.capacity).map(|slot| source.size(slot)).collect(),
            slots: vec![None; source.capacity],
            revisions: vec![0; source.capacity],
            dirty_slots: vec![false; source.capacity],
            dirty: Vec::new(),
            active: Vec::new(),
            source,
            capacity,
            reason: None,
        };
        // The fixed allocation partition is known without GPU readback. Never
        // instantiate a truncated set of blockers when the whole pool exceeds
        // the existing scene budget. No BLAS is allocated on that fallback.
        if result.source.conforming.is_some() && result.source.capacity <= capacity {
            for slot in 0..result.source.capacity {
                result.allocate(device, slot);
            }
        }
        result
    }

    fn allocate(&mut self, device: &wgpu::Device, slot: usize) {
        self.slots[slot] = Some(device.create_blas(
            &wgpu::CreateBlasDescriptor {
                label: Some("terrain-slot-blas"),
                flags: wgpu::AccelerationStructureFlags::PREFER_FAST_TRACE,
                update_mode: wgpu::AccelerationStructureUpdateMode::Build,
            },
            wgpu::BlasGeometrySizeDescriptors::Triangles {
                descriptors: vec![self.sizes[slot].clone()],
            },
        ));
    }

    /// Follow the shared encoded surface epoch, not material page IDs or
    /// bisector slot identity. No copy/readback and no second ray scene.
    pub fn sync_conforming(&mut self) {
        let Some(state) = &self.source.conforming else {
            return;
        };
        let state = state.lock().unwrap();
        let frame = state.published;
        let configuration = state.configuration_revision;
        drop(state);
        self.active.clear();
        if configuration != self.source.conforming_configuration {
            self.dirty.clear();
            self.dirty_slots.fill(false);
            self.reason = Some("RT conforming configuration changed; recreate the consumer".into());
            return;
        }
        if self.source.capacity > self.capacity {
            self.reason = Some(format!(
                "RT conforming partition has {} chunks, budget admits {}; raster terrain fallback",
                self.source.capacity, self.capacity
            ));
            return;
        }
        if frame.revision == 0 || frame.configuration_revision != configuration {
            self.reason =
                Some("RT conforming surface has not been emitted for this configuration".into());
            return;
        }
        self.reason = None;
        for slot in 0..self.source.capacity {
            self.dirty_slots[slot] |= self.revisions[slot] != frame.revision;
            self.revisions[slot] = frame.revision;
            self.active.push((
                slot,
                TerrainRayPlacement {
                    origin_m: frame.origin_m,
                    radial: [0.0; 3],
                },
            ));
        }
        self.collect_dirty();
    }

    fn collect_dirty(&mut self) {
        self.dirty.clear();
        self.dirty.extend(
            self.dirty_slots
                .iter()
                .enumerate()
                .filter_map(|(slot, &dirty)| dirty.then_some(slot)),
        );
    }

    pub fn triangle_count(&self) -> u64 {
        self.active
            .iter()
            .map(|(slot, _)| {
                if self.source.conforming.is_some() {
                    u64::from(self.sizes[*slot].vertex_count / 3)
                } else {
                    2048
                }
            })
            .sum()
    }

    pub fn sync(
        &mut self,
        backend: &Backend,
        terrain: &crate::terrain::TerrainRenderer,
    ) -> Result<(), BackendError> {
        let (vertices, ids, revisions, anchors) = terrain.ray_publication();
        if self.source.conforming.is_some() || vertices != &self.source.vertices {
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
                self.allocate(&backend.device, slot);
            }
            if self.revisions[slot] != revisions[slot] {
                self.dirty_slots[slot] = true;
            }
            self.revisions[slot] = revisions[slot];
            self.active.push((
                slot,
                TerrainRayPlacement {
                    origin_m: anchors[slot].anchor_body_m,
                    radial: anchors[slot].normal.map(|v| v as f32),
                },
            ));
        }
        self.collect_dirty();
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
                        size: &self.sizes[slot],
                        vertex_buffer: &self.source.vertices,
                        first_vertex: slot as u32
                            * if self.source.conforming.is_some() {
                                2048 * 3
                            } else {
                                VERTICES_PER_PATCH
                            },
                        vertex_stride: if self.source.conforming.is_some() {
                            crate::conforming_surface::VERTEX_BYTES
                        } else {
                            32
                        },
                        index_buffer: self.indices.as_ref(),
                        first_index: self.indices.as_ref().map(|_| 0),
                        transform_buffer: None,
                        transform_buffer_offset: None,
                    },
                ]),
            })
            .collect()
    }
}
