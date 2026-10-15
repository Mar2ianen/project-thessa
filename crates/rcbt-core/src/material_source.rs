//! Backend-neutral copy-on-write material sources and publication revisions.
//! Extracted from the MIT Bevy bridge; no renderer/ECS or game policy here.
use crate::material_page::{CbtMaterialPage, MaterialPages};
use std::{collections::BTreeSet, sync::Arc};

#[derive(Debug, Clone, Default)]
pub struct MaterialPageStore {
    generation: u64,
    priority: Vec<u64>,
    pages: Arc<MaterialPages>,
}

impl MaterialPageStore {
    pub fn generation(&self) -> u64 {
        self.generation
    }
    pub fn priority(&self) -> &[u64] {
        &self.priority
    }
    pub fn source_pages(&self) -> &MaterialPages {
        &self.pages
    }
    pub fn contains_page(&self, id: u64) -> bool {
        self.pages.contains_key(&id)
    }
    pub fn byte_len(&self) -> usize {
        self.pages.values().map(|(_, page)| page.byte_len()).sum()
    }

    /// Empty/identical updates do not invalidate render-world snapshots.
    /// Exhausted revisions fail before mutation rather than aliasing old bytes.
    pub fn set_priority(&mut self, priority: Vec<u64>) {
        if self.priority != priority {
            let next = self
                .generation
                .checked_add(1)
                .expect("material revision exhausted");
            self.priority = priority;
            self.generation = next;
        }
    }

    pub fn set_page(&mut self, id: u64, page: CbtMaterialPage) {
        self.set_pages(std::iter::once((id, page)));
    }

    /// One directory COW per completed batch; immutable mip payloads stay shared.
    pub fn set_pages(&mut self, updates: impl IntoIterator<Item = (u64, CbtMaterialPage)>) {
        let updates: Vec<_> = updates.into_iter().collect();
        if updates.is_empty() {
            return;
        }
        self.generation
            .checked_add(updates.len() as u64)
            .expect("material revision exhausted");
        let pages = Arc::make_mut(&mut self.pages);
        for (id, page) in updates {
            self.generation += 1;
            pages.insert(id, (self.generation, page));
        }
    }

    pub fn remove_page(&mut self, id: u64) {
        self.remove_pages([id]);
    }

    pub fn remove_pages(&mut self, ids: impl IntoIterator<Item = u64>) {
        let removed: BTreeSet<_> = ids
            .into_iter()
            .filter(|id| self.pages.contains_key(id))
            .collect();
        if removed.is_empty() {
            return;
        }
        let next = self
            .generation
            .checked_add(removed.len() as u64)
            .expect("material revision exhausted");
        let pages = Arc::make_mut(&mut self.pages);
        for id in removed {
            pages.remove(&id);
        }
        self.generation = next;
    }

    /// Retain nearest resident sources plus the next ancestor for blending.
    /// Under pressure the ordered hierarchy is truncated to the explicit budget.
    pub fn trim_to_budget(&mut self, budget: usize) {
        if self.pages.len() <= budget {
            return;
        }
        let pinned: BTreeSet<_> = crate::material_cache::resident_material_hierarchy(
            self.priority.iter().copied(),
            |id| self.pages.contains_key(&id),
        )
        .into_iter()
        .take(budget)
        .collect();
        let mut cold: Vec<_> = self
            .pages
            .iter()
            .filter(|(id, _)| !pinned.contains(id))
            .map(|(&id, &(generation, _))| (generation, id))
            .collect();
        cold.sort_unstable();
        let count = self.pages.len().saturating_sub(budget);
        self.remove_pages(cold.into_iter().take(count).map(|(_, id)| id));
    }

    /// Derive a backend storage representation (e.g. compact-decoded shadows)
    /// while retaining source identities, revisions and priority. Source state
    /// is untouched; only the renderer owns the derived representation.
    pub fn map_payloads(
        &self,
        mut map: impl FnMut(u64, &CbtMaterialPage) -> CbtMaterialPage,
    ) -> Self {
        Self {
            generation: self.generation,
            priority: self.priority.clone(),
            pages: Arc::new(
                self.pages
                    .iter()
                    .map(|(&id, (generation, page))| (id, (*generation, map(id, page))))
                    .collect(),
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn page(value: u8) -> CbtMaterialPage {
        CbtMaterialPage::from_rgba8(vec![value; 128 * 128 * 4]).unwrap()
    }

    #[test]
    fn snapshots_share_until_dirty_and_noops_preserve_identity() {
        let mut store = MaterialPageStore::default();
        store.set_pages([(8, page(10)), (32, page(20))]);
        store.set_priority(vec![32]);
        let old = store.clone();
        store.set_pages([]);
        store.set_priority(vec![32]);
        store.remove_pages([99, 99]);
        store.trim_to_budget(2);
        assert_eq!(store.generation(), old.generation());
        assert!(Arc::ptr_eq(&store.pages, &old.pages));
        store.set_page(32, page(40));
        assert!(!Arc::ptr_eq(&store.pages, &old.pages));
        assert_eq!(old.source_pages()[&32].1.mips()[0][0], 20);
        assert_eq!(store.source_pages()[&32].1.mips()[0][0], 40);
        assert!(std::ptr::eq(
            store.pages[&8].1.mips(),
            old.pages[&8].1.mips()
        ));
        let previous = store.pages[&32].0;
        store.remove_pages([32, 32]);
        store.set_page(32, page(60));
        assert!(store.pages[&32].0 > previous);
    }

    #[test]
    fn deterministic_trim_keeps_hierarchy_and_supports_zero_budget() {
        let mut store = MaterialPageStore::default();
        store.set_page(8, page(1));
        store.set_pages((32..48).map(|id| (id, page(2))));
        store.set_priority(vec![32]);
        store.trim_to_budget(4);
        assert_eq!(
            store.pages.keys().copied().collect::<Vec<_>>(),
            [8, 32, 46, 47]
        );
        store.trim_to_budget(0);
        assert!(store.source_pages().is_empty());
    }

    #[test]
    fn derived_representation_preserves_versions_without_mutating_source() {
        let mut source = MaterialPageStore::default();
        source.set_page(8, page(10));
        source.set_priority(vec![8]);
        let shadow = source.map_payloads(|_, _| page(11));
        assert_eq!(shadow.generation(), source.generation());
        assert_eq!(shadow.priority(), source.priority());
        assert_eq!(shadow.pages[&8].0, source.pages[&8].0);
        assert_eq!(source.pages[&8].1.mips()[0][0], 10);
        assert_eq!(shadow.pages[&8].1.mips()[0][0], 11);
    }

    #[test]
    fn revision_exhaustion_rejects_whole_batch_before_mutation() {
        let mut source = MaterialPageStore::default();
        source.set_page(8, page(1));
        source.generation = u64::MAX - 1;
        let old = source.clone();
        assert!(
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                source.set_pages([(32, page(2)), (33, page(3))]);
            }))
            .is_err()
        );
        assert!(Arc::ptr_eq(&source.pages, &old.pages));
        assert_eq!(source.generation(), old.generation());
    }
}
