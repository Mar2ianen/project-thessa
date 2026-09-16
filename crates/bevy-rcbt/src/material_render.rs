//! Persistent bounded GPU material array. The topology only carries layer IDs.
use super::*;
use crate::{
    material_cache::{SlotCache, resolve_material_ancestor},
    material_pages::{CbtRenderMaterialPages, MATERIAL_PAGE_SIZE},
};
use bevy::render::render_resource::{
    AddressMode, Extent3d, FilterMode, MipmapFilterMode, Origin3d, Sampler, SamplerDescriptor,
    TexelCopyBufferLayout, TexelCopyTextureInfo, Texture, TextureAspect, TextureDescriptor,
    TextureDimension, TextureUsages, TextureView, TextureViewDescriptor,
};

pub(super) struct MaterialArray {
    pub view: TextureView,
    pub sampler: Sampler,
    pub slots: RawBufferVec<[u32; 4]>,
    texture: Texture,
    cache: SlotCache,
    topology_generation: u64,
    pages_generation: u64,
}

impl MaterialArray {
    pub fn new(device: &RenderDevice) -> Self {
        let layers = device.limits().max_texture_array_layers.min(256);
        let texture = device.create_texture(&TextureDescriptor {
            label: Some("cbt-material-array"),
            size: Extent3d {
                width: MATERIAL_PAGE_SIZE,
                height: MATERIAL_PAGE_SIZE,
                depth_or_array_layers: layers,
            },
            mip_level_count: 8,
            sample_count: 1,
            dimension: TextureDimension::D2,
            format: TextureFormat::Rgba8UnormSrgb,
            usage: TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let view = texture.create_view(&TextureViewDescriptor {
            dimension: Some(TextureViewDimension::D2Array),
            ..Default::default()
        });
        let sampler = device.create_sampler(&SamplerDescriptor {
            label: Some("cbt-material-sampler"),
            address_mode_u: AddressMode::ClampToEdge,
            address_mode_v: AddressMode::ClampToEdge,
            mag_filter: FilterMode::Linear,
            min_filter: FilterMode::Linear,
            mipmap_filter: MipmapFilterMode::Linear,
            anisotropy_clamp: 4,
            ..Default::default()
        });
        Self {
            texture,
            view,
            sampler,
            slots: RawBufferVec::new(BufferUsages::STORAGE),
            cache: SlotCache::new(layers as usize).expect("wgpu supports texture array layers"),
            topology_generation: u64::MAX,
            pages_generation: u64::MAX,
        }
    }

    pub fn prepare(
        &mut self,
        topology: &CbtRenderTopology,
        pages: &CbtRenderMaterialPages,
        device: &RenderDevice,
        queue: &RenderQueue,
    ) {
        if self.topology_generation == topology.generation()
            && self.pages_generation == pages.generation
        {
            return;
        }
        let mut records: Vec<_> = topology.records().iter().collect();
        records.sort_by_key(|r| {
            (
                std::cmp::Reverse(r[2]),
                u64::from(r[0]) | (u64::from(r[1]) << 32),
            )
        });
        let desired: Vec<_> = records
            .iter()
            .filter_map(|r| {
                let id = u64::from(r[0]) | (u64::from(r[1]) << 32);
                let (source, _) =
                    resolve_material_ancestor(id, |key| pages.pages.contains_key(&key))?;
                Some((source, pages.pages[&source].0))
            })
            .collect();
        for change in self.cache.update(&desired) {
            let (_, page) = &pages.pages[&change.node_id];
            for (level, data) in page.mips.iter().enumerate() {
                let size = MATERIAL_PAGE_SIZE >> level;
                queue.write_texture(
                    TexelCopyTextureInfo {
                        texture: &self.texture,
                        mip_level: level as u32,
                        origin: Origin3d {
                            x: 0,
                            y: 0,
                            z: change.slot,
                        },
                        aspect: TextureAspect::All,
                    },
                    data,
                    TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(size * 4),
                        rows_per_image: Some(size),
                    },
                    Extent3d {
                        width: size,
                        height: size,
                        depth_or_array_layers: 1,
                    },
                );
            }
        }
        self.slots.clear();
        self.slots.extend(topology.records().iter().map(|r| {
            let id = u64::from(r[0]) | (u64::from(r[1]) << 32);
            match resolve_material_ancestor(id, |key| self.cache.slot(key).is_some()) {
                Some((source, uv)) => [
                    self.cache.slot(source).unwrap(),
                    uv[0].to_bits(),
                    uv[1].to_bits(),
                    uv[2].to_bits(),
                ],
                None => [u32::MAX, 1.0f32.to_bits(), 0, 0],
            }
        }));
        if topology.records().is_empty() {
            self.slots.push([u32::MAX, 1.0f32.to_bits(), 0, 0]);
        }
        self.slots.write_buffer(device, queue);
        self.topology_generation = topology.generation();
        self.pages_generation = pages.generation;
    }
}
