//! Shared height residency, bounded jobs and eviction lifecycle. Renderer
//! payloads are opaque: only the host converts built data into resident assets.
use crate::{
    JobHandle,
    selection::{self, SelectionView},
};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{Arc, mpsc::TryRecvError},
};
use thessa_rcbt_core::{HeightPage, height_source::HeightPageStore};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::{self, TileKey},
    streaming::lru_evictions,
};

#[cfg(test)]
#[path = "height_tests.rs"]
mod tests;

pub struct HeightBuild<B> {
    pub page: HeightPage,
    pub payload: B,
}

pub struct HeightEntry<'a> {
    pub generation: u64,
    pub page: &'a HeightPage,
}

struct Resident<R> {
    last_used: u64,
    payload: R,
}

pub struct HeightPoll<R> {
    pub ready: usize,
    pub changed: Vec<TileKey>,
    pub failed: Vec<(TileKey, String)>,
    /// Replaced host assets must be released too, not merely dropped.
    pub retired: Vec<(TileKey, R)>,
}

pub struct HeightAdmission<R> {
    pub spawned: usize,
    pub capacity_rejected: bool,
    pub retired: Vec<(TileKey, R)>,
}

#[derive(Clone, Copy)]
pub struct HeightPolicy {
    pub capacity: usize,
    pub max_jobs: usize,
    pub ancestors: bool,
    pub pin_roots: bool,
}

pub struct HeightStream<R = (), B = ()> {
    source: HeightPageStore,
    resident: BTreeMap<TileKey, Resident<R>>,
    pub(crate) jobs: BTreeMap<TileKey, JobHandle<Result<HeightBuild<B>, String>>>,
}

impl<R, B> Default for HeightStream<R, B> {
    fn default() -> Self {
        Self {
            source: HeightPageStore::default(),
            resident: BTreeMap::new(),
            jobs: BTreeMap::new(),
        }
    }
}

impl<R, B> HeightStream<R, B> {
    pub fn source(&self) -> &HeightPageStore {
        &self.source
    }
    pub fn len(&self) -> usize {
        self.resident.len()
    }
    pub fn is_empty(&self) -> bool {
        self.resident.is_empty()
    }
    pub fn pending_count(&self) -> usize {
        self.jobs.len()
    }
    pub fn contains_key(&self, key: &TileKey) -> bool {
        self.resident.contains_key(key)
    }
    pub fn get(&self, key: &TileKey) -> Option<&R> {
        self.resident.get(key).map(|r| &r.payload)
    }
    pub fn values(&self) -> impl Iterator<Item = &R> {
        self.resident.values().map(|r| &r.payload)
    }
    pub fn height(&self, key: TileKey) -> Option<HeightEntry<'_>> {
        let id = lod::cbt_node_for_tile(key)?.id();
        Some(HeightEntry {
            generation: self.source.page_version(id)?,
            page: self.source.get(id)?,
        })
    }

    /// Bootstrap or host-supplied ready batch. Address validation precedes host
    /// conversion; insertion/replacement returns the old opaque payload exactly once.
    pub fn insert_batch(
        &mut self,
        epoch: u64,
        builds: impl IntoIterator<Item = (TileKey, HeightBuild<B>)>,
        mut present: impl FnMut(TileKey, B) -> R,
    ) -> HeightPoll<R> {
        let mut result = HeightPoll {
            ready: 0,
            changed: Vec::new(),
            failed: Vec::new(),
            retired: Vec::new(),
        };
        let mut pages = Vec::new();
        let mut previous = Vec::new();
        for (key, build) in builds {
            let Some(node) = lod::cbt_node_for_tile(key) else {
                result
                    .failed
                    .push((key, "invalid height tile address".into()));
                continue;
            };
            pages.push((node.id(), build.page));
            previous.push((key, self.source.page_version(node.id())));
            let resident = Resident {
                last_used: epoch,
                payload: present(key, build.payload),
            };
            if let Some(old) = self.resident.insert(key, resident) {
                result.retired.push((key, old.payload));
            }
            result.ready += 1;
        }
        self.source.set_pages(pages);
        result.changed = previous
            .into_iter()
            .filter_map(|(key, version)| {
                let id = lod::cbt_node_for_tile(key)?.id();
                (self.source.page_version(id) != version).then_some(key)
            })
            .collect();
        result
    }

    /// Poll all bounded mailboxes, releasing failed/disconnected slots. A failed
    /// build does not drop other completed results or poison the drawn cover.
    pub fn poll_with(&mut self, epoch: u64, present: impl FnMut(TileKey, B) -> R) -> HeightPoll<R> {
        let mut completed = Vec::new();
        let mut failed = Vec::new();
        for (&key, job) in &self.jobs {
            match job.try_take() {
                Ok(Ok(build)) => completed.push((key, build)),
                Ok(Err(error)) => failed.push((key, error)),
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    failed.push((key, "height worker disconnected".into()))
                }
            }
        }
        for key in completed
            .iter()
            .map(|(key, _)| key)
            .chain(failed.iter().map(|(key, _)| key))
        {
            self.jobs.remove(key);
        }
        let mut result = self.insert_batch(epoch, completed, present);
        result.failed.extend(failed);
        result
    }

    /// Pin the drawn cover, demand dependencies, bootstrap roots (if requested)
    /// and pending jobs. Cold pages are evicted before new admission. Residents
    /// plus pending results never grow past capacity through this admission path;
    /// an externally supplied over-budget protected set is retained and reported.
    #[allow(clippy::too_many_arguments)]
    pub fn reconcile(
        &mut self,
        field: &Arc<PlanetField>,
        view: SelectionView,
        wanted: &[TileKey],
        visible: impl IntoIterator<Item = TileKey>,
        epoch: u64,
        policy: HeightPolicy,
        mut spawn: impl FnMut(TileKey) -> JobHandle<Result<HeightBuild<B>, String>>,
    ) -> HeightAdmission<R> {
        let dependencies =
            selection::spawn_dependencies(wanted, policy.ancestors, |key| self.contains_key(&key));
        let protected: BTreeSet<_> = visible
            .into_iter()
            .chain(dependencies.iter().copied())
            .chain((0..6).filter(|_| policy.pin_roots).map(TileKey::root))
            .chain(self.jobs.keys().copied())
            .collect();
        for key in &protected {
            if let Some(entry) = self.resident.get_mut(key) {
                entry.last_used = epoch;
            }
        }
        let entries: Vec<_> = self
            .resident
            .iter()
            .map(|(&key, entry)| (key, entry.last_used))
            .collect();
        let reserve = policy
            .capacity
            .saturating_sub(self.jobs.len() + policy.max_jobs);
        let mut result = HeightAdmission {
            spawned: 0,
            capacity_rejected: false,
            retired: Vec::new(),
        };
        let evicted = lru_evictions(&entries, &protected, reserve);
        self.source.remove_pages(
            evicted
                .iter()
                .filter_map(|&key| lod::cbt_node_for_tile(key).map(|n| n.id())),
        );
        for key in evicted {
            if let Some(entry) = self.resident.remove(&key) {
                result.retired.push((key, entry.payload));
            }
        }
        let mut pending: Vec<_> = dependencies
            .into_iter()
            .filter(|key| !self.contains_key(key) && !self.jobs.contains_key(key))
            .collect();
        pending.sort_by_cached_key(|key| selection::height_priority(field, view, *key));
        let occupied = self.resident.len().saturating_add(self.jobs.len());
        let available = policy.capacity.saturating_sub(occupied);
        result.capacity_rejected =
            occupied > policy.capacity || (!pending.is_empty() && available == 0);
        for key in pending.into_iter().take(
            policy
                .max_jobs
                .saturating_sub(self.jobs.len())
                .min(available),
        ) {
            self.jobs.insert(key, spawn(key));
            result.spawned += 1;
        }
        result
    }
}

pub fn spawn_height_job(
    field: Arc<PlanetField>,
    key: TileKey,
) -> JobHandle<Result<HeightBuild<()>, String>> {
    JobHandle::spawn(move || bake_page(&field, key).map(|page| HeightBuild { page, payload: () }))
}

/// The existing GPU-page contract: 33x33 canonical samples and 0.5 m absolute
/// quantization budget. CPU fallback meshes retain their existing builder.
pub fn bake_page(field: &PlanetField, key: TileKey) -> Result<HeightPage, String> {
    lod::bake_height_page(field, key, 33, 0.5).map_err(|error| error.to_string())
}
