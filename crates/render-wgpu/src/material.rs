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
    /// Optional linear RG8 tangent-space normal plane. Pages without a normal
    /// layer are absent from `normal_directory` and resolve to a flat normal.
    pub normal_texture: wgpu::TextureView,
    pub sampler: wgpu::Sampler,
    pub directory: wgpu::Buffer,
    pub normal_directory: wgpu::Buffer,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct MaterialUpload {
    /// Texture bytes or sampling-directory membership changed.
    pub changed: bool,
    pub pages: usize,
    /// Pages that also uploaded a normal plane.
    pub normal_pages: usize,
    pub bytes: usize,
    pub residents: usize,
    /// Actual GPU directory entries by cube-tile level (not CPU demand).
    pub resident_levels: [u32; 32],
}

pub(crate) struct MaterialArray {
    pub view: wgpu::TextureView,
    pub normal_view: wgpu::TextureView,
    pub sampler: wgpu::Sampler,
    pub slots: wgpu::Buffer,
    pub directory: wgpu::Buffer,
    pub normal_directory: wgpu::Buffer,
    texture: wgpu::Texture,
    normal_texture: wgpu::Texture,
    normal_maps: bool,
    cache: SlotCache,
    directory_data: Vec<[u32; 4]>,
    normal_directory_data: Vec<[u32; 4]>,
    slot_data: Vec<[u32; 4]>,
}

impl MaterialArray {
    pub fn binding(&self) -> MaterialBinding {
        MaterialBinding {
            texture: self.view.clone(),
            normal_texture: self.normal_view.clone(),
            sampler: self.sampler.clone(),
            directory: self.directory.clone(),
            normal_directory: self.normal_directory.clone(),
        }
    }
    pub fn new(
        backend: &Backend,
        layers: u32,
        geometry_count: usize,
        normal_maps: bool,
        anisotropy: u16,
    ) -> Result<Self, BackendError> {
        let limits = backend.device.limits();
        if !(1..=16).contains(&anisotropy)
            || (anisotropy > 1 && backend.caps.terrain_anisotropy_supported != Some(true))
        {
            return Err(BackendError::Device(
                "material anisotropy invalid or unsupported; resolve graphics settings first"
                    .into(),
            ));
        }
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
        let normal_texture = backend.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("owned-cbt-material-normal-array"),
            size: wgpu::Extent3d {
                width: if normal_maps { MATERIAL_PAGE_SIZE } else { 1 },
                height: if normal_maps { MATERIAL_PAGE_SIZE } else { 1 },
                depth_or_array_layers: if normal_maps { layers } else { 1 },
            },
            mip_level_count: if normal_maps { 8 } else { 1 },
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rg8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let normal_view = normal_texture.create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            ..Default::default()
        });
        let sampler = backend.device.create_sampler(&wgpu::SamplerDescriptor {
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            anisotropy_clamp: anisotropy,
            ..Default::default()
        });
        let cache =
            SlotCache::new(layers as usize).map_err(|e| BackendError::Device(e.to_string()))?;
        let directory_data = material_directory_with_capacity([], layers as usize).unwrap();
        let normal_directory_data = if normal_maps {
            material_directory_with_capacity([], layers as usize).unwrap()
        } else {
            vec![[0; 4]]
        };
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
        let normal_directory = buffer("owned-material-normal-directory", &normal_directory_data);
        let slots = buffer("owned-material-slots", &slot_data);
        Ok(Self {
            view,
            normal_view,
            sampler,
            slots,
            directory,
            normal_directory,
            texture,
            normal_texture,
            normal_maps,
            cache,
            directory_data,
            normal_directory_data,
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
        if pages.iter().any(|(&id, (_, page))| {
            !valid_node(id)
                || page.mips().len() != 8
                || (self.normal_maps && !complete_normal_chain(page))
        }) {
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
            if self.normal_maps
                && let Some(normal) = page.normal_mips()
            {
                for (level, data) in normal.iter().enumerate() {
                    let size = MATERIAL_PAGE_SIZE >> level;
                    backend.queue.write_texture(
                        wgpu::TexelCopyTextureInfo {
                            texture: &self.normal_texture,
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
                            bytes_per_row: Some(size * 2),
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
                stats.normal_pages += 1;
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
        for entry in &entries {
            let level = (63 - entry.node_id.leading_zeros() - 3) / 2;
            stats.resident_levels[level as usize] += 1;
        }
        let normal_directory = if self.normal_maps {
            let normal_entries: Vec<_> = entries
                .iter()
                .copied()
                .filter(|entry| {
                    pages
                        .get(&entry.node_id)
                        .is_some_and(|(_, page)| page.normal_mips().is_some())
                })
                .collect();
            material_directory_with_capacity(normal_entries, self.cache.capacity()).unwrap()
        } else {
            vec![[0; 4]]
        };
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
        if normal_directory != self.normal_directory_data {
            stats.changed = true;
            backend
                .queue
                .write_buffer(&self.normal_directory, 0, &words(&normal_directory));
            self.normal_directory_data = normal_directory;
        }
        if slots != self.slot_data {
            backend.queue.write_buffer(&self.slots, 0, &words(&slots));
            self.slot_data = slots;
        }
        stats.changed |= stats.pages > 0;
        Ok(stats)
    }
}

/// The raw texture-array backend needs every mip level present. A page
/// without a normal plane is valid and simply absent from the normal
/// directory, so the shader falls back to the flat geometric normal.
fn complete_normal_chain(page: &thessa_rcbt_core::material_page::CbtMaterialPage) -> bool {
    let Some(mips) = page.normal_mips() else {
        return true;
    };
    mips.len() == 8
        && mips.iter().enumerate().all(|(level, data)| {
            let size = (MATERIAL_PAGE_SIZE >> level) as usize;
            data.len() == size * size * 2
        })
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
    fn disabled_normals_allocate_only_dummy_and_never_upload() {
        let backend = match Backend::bring_up() {
            Ok(b) => b,
            Err(BackendError::NoAdapter) => {
                eprintln!("SKIP: no GPU");
                return;
            }
            Err(e) => panic!("{e}"),
        };
        let mut array = MaterialArray::new(&backend, 16, 1, false, 1).unwrap();
        assert_eq!(
            array.normal_texture.size(),
            wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1
            }
        );
        assert_eq!(array.normal_texture.mip_level_count(), 1);
        assert_eq!(array.normal_directory.size(), 16);
        let color = CbtMaterialPage::from_rgba8(vec![128; 128 * 128 * 4]).unwrap();
        // Decoded pages may omit coarse normal levels. Disabled consumers must
        // not reject data they neither upload nor sample.
        let page = CbtMaterialPage::from_decoded_mips_with_normals(
            color.mips().to_vec(),
            Some(vec![[200, 128].repeat(128 * 128)]),
        )
        .unwrap();
        let pages = MaterialPages::from([(12, (1, page))]);
        let stats = array.prepare(&backend, &[12], &[12], &pages).unwrap();
        assert_eq!(stats.normal_pages, 0);
        assert_eq!(stats.bytes, 87_380);
        assert!(
            array
                .normal_directory_data
                .iter()
                .all(|entry| *entry == [0; 4])
        );
        let mut enabled = MaterialArray::new(&backend, 16, 1, true, 1).unwrap();
        assert!(enabled.prepare(&backend, &[12], &[12], &pages).is_err());
    }

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
        assert!(MaterialArray::new(&backend, 0, 2, true, 1).is_err());
        for invalid in [0, 17] {
            assert!(MaterialArray::new(&backend, 2, 2, false, invalid).is_err());
        }
        assert!(
            MaterialArray::new(
                &backend,
                backend.device.limits().max_texture_array_layers + 1,
                2,
                true,
                1
            )
            .is_err()
        );
        let mut array = MaterialArray::new(&backend, 2, 2, true, 1).unwrap();
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
