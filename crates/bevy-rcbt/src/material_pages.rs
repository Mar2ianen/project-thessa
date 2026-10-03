//! Immutable material pages; RGB is sRGB albedo and alpha is linear roughness.
use bevy::prelude::Resource;
use std::{collections::BTreeMap, sync::Arc};

pub const MATERIAL_PAGE_SIZE: u32 = 128;

#[derive(Debug, Clone)]
pub struct CbtMaterialPage {
    pub(crate) mips: Arc<[Vec<u8>]>,
}

impl CbtMaterialPage {
    /// Generate the small mip chain on the streaming worker, never the render
    /// thread. RGB averages in linear light; roughness alpha stays linear.
    pub fn from_rgba8(rgba: Vec<u8>) -> Option<Self> {
        if rgba.len() != (MATERIAL_PAGE_SIZE * MATERIAL_PAGE_SIZE * 4) as usize {
            return None;
        }
        let linear: [f32; 256] = std::array::from_fn(|i| {
            let s = i as f32 / 255.0;
            if s <= 0.04045 {
                s / 12.92
            } else {
                ((s + 0.055) / 1.055).powf(2.4)
            }
        });
        let mut mips = vec![rgba];
        let mut size = MATERIAL_PAGE_SIZE as usize;
        while size > 1 {
            let source = mips.last().unwrap();
            let next_size = size / 2;
            let mut next = vec![0; next_size * next_size * 4];
            for y in 0..next_size {
                for x in 0..next_size {
                    for c in 0..4 {
                        let mut sum = 0.0;
                        for dy in 0..2 {
                            for dx in 0..2 {
                                let value = source[((y * 2 + dy) * size + x * 2 + dx) * 4 + c];
                                sum += if c < 3 {
                                    linear[value as usize]
                                } else {
                                    value as f32 / 255.0
                                };
                            }
                        }
                        let avg = sum * 0.25;
                        let encoded = if c == 3 {
                            avg
                        } else if avg <= 0.0031308 {
                            12.92 * avg
                        } else {
                            1.055 * avg.powf(1.0 / 2.4) - 0.055
                        };
                        next[(y * next_size + x) * 4 + c] =
                            (encoded * 255.0).round().clamp(0.0, 255.0) as u8;
                    }
                }
            }
            mips.push(next);
            size = next_size;
        }
        Some(Self { mips: mips.into() })
    }
    pub fn byte_len(&self) -> usize {
        self.mips.iter().map(Vec::len).sum()
    }

    /// Mip levels, finest first. Level 0 is the full 128x128 RGBA page;
    /// each next level halves both extents down to 1x1.
    pub fn mips(&self) -> &[Vec<u8>] {
        &self.mips
    }

    /// Rebuild a page from already-sized mip levels (microstore decode
    /// path): validates the 128-halving shape instead of averaging.
    /// Returns `None` on shape mismatch, like [`CbtMaterialPage::from_rgba8`].
    pub fn from_decoded_mips(mips: Vec<Vec<u8>>) -> Option<Self> {
        if mips.is_empty() || mips.len() > 8 {
            return None;
        }
        let mut size = MATERIAL_PAGE_SIZE as usize;
        for mip in &mips {
            if mip.len() != size * size * 4 {
                return None;
            }
            size = (size / 2).max(1);
        }
        Some(Self { mips: mips.into() })
    }
}

#[derive(Debug, Clone, Default, Resource)]
pub struct CbtRenderMaterialPages {
    pub(crate) generation: u64,
    pub(crate) priority: Vec<u64>,
    /// Extraction shares the immutable page directory. Stream batches use
    /// copy-on-write, while each page's mip payload is already Arc-backed.
    pub(crate) pages: Arc<BTreeMap<u64, (u64, CbtMaterialPage)>>,
}

impl CbtRenderMaterialPages {
    /// Ordered visible/prefetch sources, independent from topology ordering.
    pub fn set_priority(&mut self, priority: Vec<u64>) {
        if self.priority != priority {
            self.priority = priority;
            self.generation = self.generation.saturating_add(1);
        }
    }

    /// Immutable view of the current priority. Check this through a shared
    /// (`Res`/`&`) borrow *before* taking a mutable borrow: calling any
    /// `&mut self` method via Bevy `ResMut` marks the resource as changed
    /// (via `DerefMut`) even when the bytes end up identical, which forces
    /// `ExtractResourcePlugin` to publish a new render-world snapshot. The
    /// `&self` path uses `Deref` only and leaves the change flag alone.
    pub fn priority(&self) -> &[u64] {
        &self.priority
    }

    pub fn byte_len(&self) -> usize {
        self.pages.values().map(|(_, page)| page.byte_len()).sum()
    }
    pub fn contains_page(&self, node_id: u64) -> bool {
        self.pages.contains_key(&node_id)
    }
    /// Bound independent material residency, retaining demand's nearest
    /// resident source and its next resident ancestor for level blending.
    /// Generation is the deterministic cold-page eviction order.
    pub fn trim_to_budget(&mut self, budget: usize) {
        if self.pages.len() <= budget {
            return;
        }
        let mut pinned = std::collections::BTreeSet::new();
        for &id in &self.priority {
            let mut ancestor = id;
            let mut retained = 0;
            while ancestor >= 8 {
                if self.pages.contains_key(&ancestor) && pinned.len() < budget {
                    pinned.insert(ancestor);
                    retained += 1;
                    if retained == 2 {
                        break;
                    }
                }
                ancestor >>= 2;
            }
        }
        let mut cold: Vec<_> = self
            .pages
            .iter()
            .filter(|(id, _)| !pinned.contains(*id))
            .map(|(&id, &(generation, _))| (generation, id))
            .collect();
        cold.sort_unstable();
        let count = self.pages.len().saturating_sub(budget);
        self.remove_pages(cold.into_iter().take(count).map(|(_, id)| id));
    }
    pub fn set_page(&mut self, node_id: u64, page: CbtMaterialPage) {
        self.set_pages(std::iter::once((node_id, page)));
    }

    /// Publish completed material pages with one copy-on-write directory
    /// update, rather than copying the directory once per completed worker.
    pub fn set_pages(&mut self, updates: impl IntoIterator<Item = (u64, CbtMaterialPage)>) {
        let updates: Vec<_> = updates.into_iter().collect();
        if updates.is_empty() {
            return;
        }
        let pages = Arc::make_mut(&mut self.pages);
        for (node_id, page) in updates {
            self.generation = self.generation.saturating_add(1);
            pages.insert(node_id, (self.generation, page));
        }
    }

    pub fn remove_page(&mut self, node_id: u64) {
        self.remove_pages(std::iter::once(node_id));
    }

    /// Remove multiple evicted material pages with one directory update.
    pub fn remove_pages(&mut self, node_ids: impl IntoIterator<Item = u64>) {
        let removed: Vec<_> = node_ids
            .into_iter()
            .filter(|node_id| self.pages.contains_key(node_id))
            .collect();
        if removed.is_empty() {
            return;
        }
        let pages = Arc::make_mut(&mut self.pages);
        for node_id in removed {
            if pages.remove(&node_id).is_some() {
                self.generation = self.generation.saturating_add(1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn independent_material_residency_retains_demand_and_ancestors() {
        let page = CbtMaterialPage::from_rgba8(vec![128; 128 * 128 * 4]).unwrap();
        let mut pages = CbtRenderMaterialPages::default();
        pages.set_page(8, page.clone());
        for id in 32..48 {
            pages.set_page(id, page.clone());
        }
        pages.set_priority(vec![32]);
        pages.trim_to_budget(4);
        assert_eq!(pages.pages.len(), 4);
        assert!(pages.contains_page(8));
        assert!(pages.contains_page(32));
    }

    #[test]
    fn mips_average_color_in_linear_light_and_keep_roughness_linear() {
        let mut bytes = vec![0; 128 * 128 * 4];
        for (i, p) in bytes.as_chunks_mut::<4>().0.iter_mut().enumerate() {
            let value = if i % 2 == 0 { 0 } else { 255 };
            p.copy_from_slice(&[value; 4]);
        }
        let page = CbtMaterialPage::from_rgba8(bytes).unwrap();
        assert_eq!(page.mips.len(), 8);
        assert_eq!(&page.mips[1][..4], &[188, 188, 188, 128]);
        assert_eq!(page.mips.last().unwrap().len(), 4);
        assert!(CbtMaterialPage::from_rgba8(vec![0; 4]).is_none());
    }

    #[test]
    fn extracted_material_snapshot_shares_directory_until_batch_update() {
        let page = CbtMaterialPage::from_rgba8(vec![32; 128 * 128 * 4]).unwrap();
        let replacement = CbtMaterialPage::from_rgba8(vec![96; 128 * 128 * 4]).unwrap();
        let mut pages = CbtRenderMaterialPages::default();
        pages.set_pages([(17, page.clone()), (19, page.clone())]);
        let snapshot = pages.clone();
        assert!(Arc::ptr_eq(&pages.pages, &snapshot.pages));
        assert_eq!(snapshot.generation, 2);

        pages.set_pages([(17, replacement)]);
        assert!(!Arc::ptr_eq(&pages.pages, &snapshot.pages));
        assert_eq!(pages.generation, 3);
        assert_eq!(pages.pages[&17].0, 3);
        assert_eq!(snapshot.pages[&17].0, 1);
        assert!(Arc::ptr_eq(
            &pages.pages[&19].1.mips,
            &snapshot.pages[&19].1.mips
        ));

        pages.remove_pages([17, 19]);
        assert_eq!(pages.generation, 5);
        assert!(pages.pages.is_empty());
        assert_eq!(snapshot.pages.len(), 2);
    }
}
