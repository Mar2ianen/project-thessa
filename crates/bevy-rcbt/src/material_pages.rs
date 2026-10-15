//! Immutable material pages; RGB is sRGB albedo and alpha is linear roughness.
use bevy::prelude::Resource;
use std::{collections::BTreeMap, sync::Arc};

pub use thessa_rcbt_core::material_page::{CbtMaterialPage, MATERIAL_PAGE_SIZE};

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

    /// Borrow the backend-neutral immutable source snapshot without conversion
    /// or copying. Owned renderers consume the same pages as the Bevy host.
    pub fn source_pages(&self) -> &thessa_rcbt_core::material_page::MaterialPages {
        &self.pages
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
        let pinned: std::collections::BTreeSet<_> =
            crate::material_cache::resident_material_hierarchy(
                self.priority.iter().copied(),
                |id| self.pages.contains_key(&id),
            )
            .into_iter()
            .take(budget)
            .collect();
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
        assert_eq!(page.mips().len(), 8);
        assert_eq!(&page.mips()[1][..4], &[188, 188, 188, 128]);
        assert_eq!(page.mips().last().unwrap().len(), 4);
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
        assert!(std::ptr::eq(
            pages.pages[&19].1.mips(),
            snapshot.pages[&19].1.mips()
        ));

        pages.remove_pages([17, 19]);
        assert_eq!(pages.generation, 5);
        assert!(pages.pages.is_empty());
        assert_eq!(snapshot.pages.len(), 2);
    }
}
