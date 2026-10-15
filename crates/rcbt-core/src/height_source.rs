//! Immutable height-source snapshots, independent of the renderer and game.
use crate::HeightPage;
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

pub type HeightPages = BTreeMap<u64, (u64, Arc<HeightPage>)>;

/// Revisions are local to one producer lifetime. Reset its renderer consumer
/// when replacing the producer; clones are immutable extraction snapshots.
#[derive(Debug, Clone, Default)]
pub struct HeightPageStore {
    generation: u64,
    pages: Arc<HeightPages>,
}

impl HeightPageStore {
    pub fn generation(&self) -> u64 {
        self.generation
    }
    pub fn len(&self) -> usize {
        self.pages.len()
    }
    pub fn is_empty(&self) -> bool {
        self.pages.is_empty()
    }
    pub fn contains_page(&self, id: u64) -> bool {
        self.pages.contains_key(&id)
    }
    pub fn page_version(&self, id: u64) -> Option<u64> {
        self.pages.get(&id).map(|p| p.0)
    }
    pub fn get(&self, id: u64) -> Option<&HeightPage> {
        self.pages.get(&id).map(|p| p.1.as_ref())
    }
    pub fn source_pages(&self) -> &HeightPages {
        &self.pages
    }
    pub fn set_page(&mut self, id: u64, page: HeightPage) {
        self.set_pages([(id, page)]);
    }

    /// Content-equal publication is a no-op. A dirty batch COWs only the
    /// directory; all unchanged height vectors remain shared. Every changed
    /// identity uses the producer revision, including eviction/reinsertion.
    pub fn set_pages(&mut self, updates: impl IntoIterator<Item = (u64, HeightPage)>) {
        let mut changed = BTreeMap::new();
        let mut revision = self.generation;
        for (id, page) in updates {
            let current = changed.get(&id).or_else(|| self.pages.get(&id));
            if current.is_some_and(|(_, old): &(u64, Arc<HeightPage>)| old.as_ref() == &page) {
                continue;
            }
            revision = revision.checked_add(1).expect("height revision exhausted");
            changed.insert(id, (revision, Arc::new(page)));
        }
        if changed.is_empty() {
            return;
        }
        Arc::make_mut(&mut self.pages).extend(changed);
        self.generation = revision;
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
        let revision = self
            .generation
            .checked_add(removed.len() as u64)
            .expect("height revision exhausted");
        let pages = Arc::make_mut(&mut self.pages);
        for id in removed {
            pages.remove(&id);
        }
        self.generation = revision;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn page(value: f64) -> HeightPage {
        HeightPage::bake(&[value; 4], 2, 0.01).unwrap()
    }
    #[test]
    fn cow_content_noops_and_reinsertion_identity() {
        let mut source = HeightPageStore::default();
        source.set_pages([(8, page(1.0)), (9, page(2.0))]);
        let old = source.clone();
        source.set_pages([(8, page(1.0)), (8, page(1.0))]);
        source.remove_pages([99, 99]);
        assert_eq!(source.generation(), old.generation());
        assert!(Arc::ptr_eq(&source.pages, &old.pages));
        source.set_pages([(8, page(3.0)), (8, page(4.0))]);
        assert_eq!(source.get(8), Some(&page(4.0)));
        assert_eq!(old.get(8), Some(&page(1.0)));
        assert!(Arc::ptr_eq(&source.pages[&9].1, &old.pages[&9].1));
        let version = source.page_version(8).unwrap();
        source.remove_pages([8, 8]);
        source.set_page(8, page(5.0));
        assert!(source.page_version(8).unwrap() > version);
    }
    #[test]
    fn exhausted_batch_preserves_source() {
        let mut source = HeightPageStore::default();
        source.set_page(8, page(1.0));
        source.generation = u64::MAX - 1;
        let old = source.clone();
        assert!(
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                source.set_pages([(9, page(2.0)), (10, page(3.0))]);
            }))
            .is_err()
        );
        assert!(Arc::ptr_eq(&source.pages, &old.pages));
        assert_eq!(source.generation(), old.generation());
    }
}
