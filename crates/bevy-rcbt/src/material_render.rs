//! Persistent bounded GPU material array. The topology only carries layer IDs.
use super::*;
use crate::{
    material_cache::{SlotCache, resolve_material_ancestor},
    material_microstore::{EncodedMaterialLevel, encode_material_level},
    material_pages::{CbtRenderMaterialPages, MATERIAL_PAGE_SIZE},
};
use bevy::render::render_resource::{
    AddressMode, Extent3d, FilterMode, MipmapFilterMode, Origin3d, Sampler, SamplerDescriptor,
    TexelCopyBufferLayout, TexelCopyTextureInfo, Texture, TextureAspect, TextureDescriptor,
    TextureDimension, TextureUsages, TextureView, TextureViewDescriptor,
};
use std::collections::BTreeMap;
use std::time::Instant;
use thessa_graphics::ResolvedMaterialStorage;

/// CPU-only compact residency for the microstore storage path.
///
/// Holds encoded pages once per generation and builds decoded shadow pages
/// for the shared texture-upload path. No GPU/Bevy types cross this
/// boundary, so the render-world tests below pin it without an adapter.
/// Sampling never sees which storage produced the bytes: texture format,
/// mips, and filtering stay identical.
#[derive(Debug, Default)]
pub(crate) struct MicrostoreResidency {
    /// node id -> (page generation, encoded mip levels).
    pages: BTreeMap<u64, (u64, Vec<EncodedMaterialLevel>)>,
    /// Current compact residency in wire bytes (gauge, not lifetime).
    pub wire_bytes: u64,
    /// Lifetime RGBA bytes decoded at upload time.
    pub decoded_bytes: u64,
    /// Lifetime CPU encode seconds (page -> compact form, once per generation).
    pub encode_secs: f64,
    /// Pages ever encoded through the compact path.
    pub pages_encoded: u64,
}

impl MicrostoreResidency {
    /// Encode changed generations once, drop evicted pages, and return
    /// decoded shadow pages for the shared upload path.
    pub fn update(&mut self, pages: &CbtRenderMaterialPages) -> CbtRenderMaterialPages {
        self.pages.retain(|id, _| pages.pages.contains_key(id));
        for (id, (generation, page)) in pages.pages.iter() {
            if let Some((g, _)) = self.pages.get(id) {
                if *g == *generation {
                    continue;
                }
            }
            let started = Instant::now();
            let levels = MaterialArray::encode_page_levels(page);
            self.encode_secs += started.elapsed().as_secs_f64();
            self.pages_encoded += 1;
            self.pages.insert(*id, (*generation, levels));
        }
        self.wire_bytes = self
            .pages
            .values()
            .flat_map(|(_, levels)| levels.iter())
            .map(EncodedMaterialLevel::encoded_bytes)
            .sum::<usize>() as u64;
        let mut shadow = CbtRenderMaterialPages {
            generation: pages.generation,
            priority: pages.priority.clone(),
            pages: BTreeMap::new(),
        };
        for (id, (_, levels)) in self.pages.iter() {
            let decoded = MaterialArray::decode_material_levels(levels);
            self.decoded_bytes += decoded.iter().map(Vec::len).sum::<usize>() as u64;
            let page = crate::material_pages::CbtMaterialPage::from_decoded_mips(decoded)
                .expect("decoded levels keep mip shapes");
            shadow.pages.insert(*id, (pages.pages[id].0, page));
        }
        shadow
    }
}

pub(super) struct MaterialArray {
    pub view: TextureView,
    pub sampler: Sampler,
    pub slots: RawBufferVec<[u32; 4]>,
    texture: Texture,
    cache: SlotCache,
    topology_generation: u64,
    pages_generation: u64,
    microstore: MicrostoreResidency,
}

impl MaterialArray {
    pub fn new(device: &RenderDevice) -> Self {
        let layers = device.limits().max_texture_array_layers.min(512);
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
            microstore: MicrostoreResidency::default(),
        }
    }

    /// Telemetry accessors for the compact path (gauge + lifetimes).
    pub fn microstore_wire_bytes(&self) -> u64 {
        self.microstore.wire_bytes
    }
    pub fn microstore_decoded_bytes(&self) -> u64 {
        self.microstore.decoded_bytes
    }
    pub fn microstore_encode_secs(&self) -> f64 {
        self.microstore.encode_secs
    }
    pub fn microstore_pages_encoded(&self) -> u64 {
        self.microstore.pages_encoded
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
        let sources: Vec<_> = if pages.priority.is_empty() {
            records
                .iter()
                .map(|r| u64::from(r[0]) | (u64::from(r[1]) << 32))
                .collect()
        } else {
            pages.priority.clone()
        };
        let desired: Vec<_> = sources
            .into_iter()
            .filter_map(|id| {
                let (source, _) =
                    resolve_material_ancestor(id, |key| pages.pages.contains_key(&key))?;
                Some((source, pages.pages[&source].0))
            })
            .collect();
        for change in self.cache.update_retaining(&desired) {
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
            match resolve_material_ancestor(id, |key| {
                self.cache
                    .slot(key)
                    .and_then(|slot| self.cache.entry(slot))
                    .is_some_and(|entry| {
                        pages
                            .pages
                            .get(&key)
                            .is_some_and(|(generation, _)| *generation == entry.generation)
                    })
            }) {
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

impl MaterialArray {
    /// Upload entry point with the resolved storage policy. The raw array
    /// path is the default; the compact path encodes pages once, holds
    /// the compact form in residency, and decodes to RGBA at upload time
    /// through the identical texture, sampler, mips, and slot logic.
    pub fn prepare_storage(
        &mut self,
        topology: &CbtRenderTopology,
        pages: &CbtRenderMaterialPages,
        device: &RenderDevice,
        queue: &RenderQueue,
        storage: ResolvedMaterialStorage,
    ) {
        match storage {
            ResolvedMaterialStorage::RgbaArray => self.prepare(topology, pages, device, queue),
            ResolvedMaterialStorage::MicrostoreCompact => {
                self.prepare_compact(topology, pages, device, queue)
            }
        }
    }

    /// Decode encoded mip levels back to interleaved RGBA bytes, one vec
    /// per level. Pure function of the encoded data: the render-world test
    /// pins decoded == original within the encode budget without a GPU.
    pub fn decode_material_levels(levels: &[EncodedMaterialLevel]) -> Vec<Vec<u8>> {
        levels
            .iter()
            .map(|level| {
                let [r, g, b] = level.color.decode();
                let a = level.roughness.decode();
                let n = r.data.len();
                debug_assert_eq!(g.data.len(), n);
                debug_assert_eq!(b.data.len(), n);
                debug_assert_eq!(a.data.len(), n);
                let mut rgba = Vec::with_capacity(n * 4);
                for i in 0..n {
                    rgba.extend_from_slice(&[r.data[i], g.data[i], b.data[i], a.data[i]]);
                }
                rgba
            })
            .collect()
    }

    fn prepare_compact(
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
        // Only streaming frames reach here (generation guard above), so one
        // line per upload batch is the A/B signal: compact residency held
        // vs RGBA decoded for the shared upload, plus lifetime encode cost.
        let shadow = self.microstore.update(pages);
        self.prepare(topology, &shadow, device, queue);
        eprintln!(
            "[material-storage] microstore_compact: {} pages, wire {} B, decoded {} B lifetime, \
             encode {:.2} ms lifetime, {} pages encoded lifetime",
            self.microstore.pages.len(),
            self.microstore.wire_bytes,
            self.microstore.decoded_bytes,
            self.microstore.encode_secs * 1000.0,
            self.microstore.pages_encoded,
        );
    }

    /// Encode every mip of one page with the sample-time budget (2.0 code
    /// levels). Shared by the render path and the CPU-only tests.
    pub(crate) fn encode_page_levels(
        page: &crate::material_pages::CbtMaterialPage,
    ) -> Vec<EncodedMaterialLevel> {
        let (mut w, mut h) = (MATERIAL_PAGE_SIZE, MATERIAL_PAGE_SIZE);
        let mut levels = Vec::with_capacity(page.mips.len());
        for mip in page.mips.iter() {
            if let Some(encoded) = encode_material_level(mip, w, h, 2.0) {
                levels.push(encoded);
            }
            w = (w / 2).max(1);
            h = (h / 2).max(1);
        }
        levels
    }
}

#[cfg(test)]
mod residency_tests {
    use super::*;
    use crate::material_microstore::rock_rgba;
    use crate::material_pages::CbtMaterialPage;

    fn real_pages(ids: &[u64], seed: u64) -> CbtRenderMaterialPages {
        let mut out = CbtRenderMaterialPages::default();
        for (i, id) in ids.iter().enumerate() {
            let rgba = rock_rgba(MATERIAL_PAGE_SIZE, seed + i as u64 * 0x9E37);
            let page = CbtMaterialPage::from_rgba8(rgba).expect("real page builds");
            out.set_page(*id, page);
        }
        out
    }

    #[test]
    fn decode_round_trips_within_encode_budget() {
        let rgba = rock_rgba(MATERIAL_PAGE_SIZE, 0xC0FFEE);
        let page = CbtMaterialPage::from_rgba8(rgba.clone()).expect("real page builds");
        let levels = MaterialArray::encode_page_levels(&page);
        assert_eq!(levels.len(), 8);
        let decoded = MaterialArray::decode_material_levels(&levels);
        assert_eq!(decoded.len(), 8);
        let mut worst = 0u8;
        for (level, (orig, back)) in page.mips.iter().zip(decoded.iter()).enumerate() {
            assert_eq!(orig.len(), back.len(), "level {level} len");
            for (i, (o, d)) in orig.iter().zip(back.iter()).enumerate() {
                worst = worst.max(o.abs_diff(*d));
                assert!(o.abs_diff(*d) <= 2, "level {level} byte {i}: {o} vs {d}");
            }
        }
        // Compact residency must beat the raw 87,380 B mip chain.
        let wire: usize = levels.iter().map(EncodedMaterialLevel::encoded_bytes).sum();
        let raw: usize = page.mips.iter().map(Vec::len).sum();
        eprintln!("compact {wire} B vs raw {raw} B, worst drift {worst}");
        assert!(wire < raw, "microstore must beat raw RGBA");
        // Shadow pages keep the exact mip shapes for the shared upload path.
        let rebuilt = CbtMaterialPage::from_decoded_mips(decoded).expect("shape holds");
        assert_eq!(rebuilt.mips.len(), 8);
        assert!(CbtMaterialPage::from_decoded_mips(vec![vec![0u8; 3]]).is_none());
    }

    #[test]
    fn residency_encodes_once_and_evicts() {
        let pages = real_pages(&[11, 22], 0x5EED);
        let mut cache = MicrostoreResidency::default();
        let shadow = cache.update(&pages);
        assert_eq!(shadow.pages.len(), 2);
        assert_eq!(cache.pages_encoded, 2);
        let wire_once = cache.wire_bytes;
        assert!(wire_once > 0);
        assert!(cache.decoded_bytes > 0);
        // Same generations: no re-encode, gauge stable.
        let shadow2 = cache.update(&pages);
        assert_eq!(cache.pages_encoded, 2, "must not re-encode clean pages");
        assert_eq!(cache.wire_bytes, wire_once, "wire gauge must be stable");
        assert_eq!(shadow2.pages.len(), 2);
        // Evict one page: residency shrinks, shadow follows.
        let mut fewer = pages.clone();
        fewer.remove_page(11);
        let shadow3 = cache.update(&fewer);
        assert_eq!(shadow3.pages.len(), 1);
        assert!(
            cache.wire_bytes < wire_once,
            "eviction must shrink residency"
        );
        assert!(!cache.pages.contains_key(&11));
    }
}
