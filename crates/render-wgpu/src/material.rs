//! Persistent raw material residency using the shared production page/cache ABI.
use crate::{Backend, BackendError};
use thessa_rcbt_core::{
    material_cache::{
        SlotCache, material_directory_with_capacity, resident_material_hierarchy,
        resolve_material_ancestor,
    },
    material_page::MATERIAL_PAGE_SIZE,
};

pub use thessa_rcbt_core::material_page::MaterialPages;

/// Borrowed live residency for native surface consumers. Clones share texture
/// layers and the sparse directory; geometry allocation IDs never index them.
#[derive(Debug, Clone)]
pub struct MaterialBinding {
    pub texture: wgpu::TextureView,
    pub sampler: wgpu::Sampler,
    pub directory: wgpu::Buffer,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct MaterialUpload {
    /// Texture bytes or sampling-directory membership changed.
    pub changed: bool,
    pub pages: usize,
    pub bytes: usize,
    pub residents: usize,
}

pub(crate) struct MaterialArray {
    pub view: wgpu::TextureView,
    pub sampler: wgpu::Sampler,
    pub slots: wgpu::Buffer,
    pub directory: wgpu::Buffer,
    texture: wgpu::Texture,
    cache: SlotCache,
    directory_data: Vec<[u32; 4]>,
    slot_data: Vec<[u32; 4]>,
}

impl MaterialArray {
    pub fn binding(&self) -> MaterialBinding {
        MaterialBinding {
            texture: self.view.clone(),
            sampler: self.sampler.clone(),
            directory: self.directory.clone(),
        }
    }
    pub fn new(
        backend: &Backend,
        layers: u32,
        geometry_count: usize,
    ) -> Result<Self, BackendError> {
        let limits = backend.device.limits();
        let directory_len = (u64::from(layers).max(1) * 4).next_power_of_two();
        if layers == 0
            || layers > limits.max_texture_array_layers
            || MATERIAL_PAGE_SIZE > limits.max_texture_dimension_2d
            || directory_len * 16
                > limits
                    .max_storage_buffer_binding_size
                    .min(limits.max_buffer_size)
        {
            return Err(BackendError::Device(
                "material capacity exceeds device limits".into(),
            ));
        }
        let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("owned-cbt-material-array"),
            size: wgpu::Extent3d {
                width: MATERIAL_PAGE_SIZE,
                height: MATERIAL_PAGE_SIZE,
                depth_or_array_layers: layers,
            },
            mip_level_count: 8,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            ..Default::default()
        });
        let sampler = backend.device.create_sampler(&wgpu::SamplerDescriptor {
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            ..Default::default()
        });
        let cache =
            SlotCache::new(layers as usize).map_err(|e| BackendError::Device(e.to_string()))?;
        let directory_data = material_directory_with_capacity([], layers as usize).unwrap();
        let slot_data = vec![[u32::MAX, 1.0f32.to_bits(), 0, 0]; geometry_count];
        let buffer = |label, data: &[[u32; 4]]| {
            use wgpu::util::DeviceExt;
            backend
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(label),
                    contents: &words(data),
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                })
        };
        let directory = buffer("owned-material-directory", &directory_data);
        let slots = buffer("owned-material-slots", &slot_data);
        Ok(Self {
            view,
            sampler,
            slots,
            directory,
            texture,
            cache,
            directory_data,
            slot_data,
        })
    }

    pub fn prepare(
        &mut self,
        backend: &Backend,
        geometry: &[u64],
        demand: &[u64],
        pages: &MaterialPages,
    ) -> Result<MaterialUpload, BackendError> {
        // Validate the complete publication before changing residency or GPU data.
        // Decoded microstore pages may omit coarse mips; this raw backend rejects
        // them rather than sampling zero/uninitialised coarse levels.
        if pages
            .iter()
            .any(|(&id, (_, page))| !valid_node(id) || page.mips().len() != 8)
        {
            return Err(BackendError::Device(
                "materials require cube-tile IDs and complete mip chains".into(),
            ));
        }
        if demand.iter().any(|&id| !valid_node(id)) {
            return Err(BackendError::Device("invalid material demand node".into()));
        }
        let hierarchy =
            resident_material_hierarchy(demand.iter().copied(), |id| pages.contains_key(&id));
        let requested: Vec<_> = hierarchy.into_iter().map(|id| (id, pages[&id].0)).collect();
        let changes = self.cache.update_retaining(&requested);
        let mut stats = MaterialUpload::default();
        for change in changes {
            let page = &pages[&change.node_id].1;
            for (level, data) in page.mips().iter().enumerate() {
                let size = MATERIAL_PAGE_SIZE >> level;
                backend.queue.write_texture(
                    wgpu::TexelCopyTextureInfo {
                        texture: &self.texture,
                        mip_level: level as u32,
                        origin: wgpu::Origin3d {
                            x: 0,
                            y: 0,
                            z: change.slot,
                        },
                        aspect: wgpu::TextureAspect::All,
                    },
                    data,
                    wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(size * 4),
                        rows_per_image: Some(size),
                    },
                    wgpu::Extent3d {
                        width: size,
                        height: size,
                        depth_or_array_layers: 1,
                    },
                );
                stats.bytes += data.len();
            }
            stats.pages += 1;
        }
        let current = |id| {
            self.cache
                .slot(id)
                .and_then(|slot| self.cache.entry(slot))
                .filter(|entry| {
                    pages
                        .get(&id)
                        .is_some_and(|(generation, _)| *generation == entry.generation)
                })
        };
        let entries: Vec<_> = self
            .cache
            .iter()
            .filter(|entry| current(entry.node_id).is_some())
            .collect();
        stats.residents = entries.len();
        let directory = material_directory_with_capacity(entries, self.cache.capacity()).unwrap();
        let slots: Vec<_> = geometry
            .iter()
            .map(
                |&id| match resolve_material_ancestor(id, |key| current(key).is_some()) {
                    Some((source, uv)) => [
                        current(source).unwrap().slot,
                        uv[0].to_bits(),
                        uv[1].to_bits(),
                        uv[2].to_bits(),
                    ],
                    None => [u32::MAX, 1.0f32.to_bits(), 0, 0],
                },
            )
            .collect();
        if directory != self.directory_data {
            stats.changed = true;
            backend
                .queue
                .write_buffer(&self.directory, 0, &words(&directory));
            self.directory_data = directory;
        }
        if slots != self.slot_data {
            backend.queue.write_buffer(&self.slots, 0, &words(&slots));
            self.slot_data = slots;
        }
        stats.changed |= stats.pages > 0;
        Ok(stats)
    }
}

fn words(data: &[[u32; 4]]) -> Vec<u8> {
    data.iter()
        .flatten()
        .flat_map(|v| v.to_le_bytes())
        .collect()
}

fn valid_node(mut id: u64) -> bool {
    while id >= 16 {
        id >>= 2;
    }
    (8..14).contains(&id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_rcbt_core::material_page::CbtMaterialPage;

    #[test]
    fn bounded_residency_protects_ancestors_and_never_publishes_stale_generations() {
        let backend = match Backend::bring_up() {
            Ok(b) => b,
            Err(BackendError::NoAdapter) => {
                eprintln!("SKIP: no GPU");
                return;
            }
            Err(e) => panic!("{e}"),
        };
        assert!(MaterialArray::new(&backend, 0, 2).is_err());
        assert!(
            MaterialArray::new(
                &backend,
                backend.device.limits().max_texture_array_layers + 1,
                2
            )
            .is_err()
        );
        let mut array = MaterialArray::new(&backend, 2, 2).unwrap();
        let page = CbtMaterialPage::from_rgba8(vec![128; 128 * 128 * 4]).unwrap();
        let mut pages = MaterialPages::from([
            (12, (1, page.clone())),
            (48, (1, page.clone())),
            (49, (1, page)),
        ]);
        assert_eq!(
            array
                .prepare(&backend, &[48, 49], &[48, 49], &pages)
                .unwrap()
                .pages,
            2
        );
        let parent_slot = array.cache.slot(12).unwrap();
        assert_eq!(
            array.slot_data[1],
            [parent_slot, 0.5f32.to_bits(), 0, 0.5f32.to_bits()]
        );
        assert_eq!(array.directory_data.len(), 8);
        assert_eq!(
            array
                .prepare(&backend, &[48, 49], &[49, 48], &pages)
                .unwrap()
                .pages,
            1
        );
        assert_eq!(array.cache.slot(12), Some(parent_slot));
        assert_eq!(array.cache.slot(48), None);
        assert_eq!(
            array
                .prepare(&backend, &[48, 49], &[49, 48], &pages)
                .unwrap()
                .pages,
            0
        );
        // Source changes while not requested: old warm content must disappear
        // from the published directory until actually uploaded at new generation.
        pages.get_mut(&49).unwrap().0 = 2;
        let stats = array.prepare(&backend, &[48, 49], &[12], &pages).unwrap();
        assert_eq!((stats.pages, stats.residents), (0, 1));
        assert_eq!(array.slot_data[1][0], parent_slot);
        assert_eq!(
            array
                .prepare(&backend, &[48, 49], &[49], &pages)
                .unwrap()
                .pages,
            1
        );
        assert_eq!(
            array
                .prepare(&backend, &[48, 49], &[12], &pages)
                .unwrap()
                .pages,
            0
        );
        assert_eq!(
            array
                .prepare(&backend, &[48, 49], &[49], &pages)
                .unwrap()
                .pages,
            0,
            "warm revisit must not reupload"
        );
    }
}
