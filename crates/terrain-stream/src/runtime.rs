//! Renderer-free execution of the extracted terrain policy on the Rayon pool.
//! Hosts publish snapshots only when coverage/material state changes.
#[cfg(test)]
#[path = "runtime_tests.rs"]
mod tests;
use crate::{
    JobHandle,
    selection::{self, SelectionHistory, SelectionPolicy, SelectionView},
};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{Arc, mpsc::TryRecvError},
};
use thessa_rcbt_core::{
    HeightPage,
    material_cache::resident_material_hierarchy,
    material_page::{CbtMaterialPage, MaterialPages},
};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::{self, TileKey},
    streaming::{lru_evictions, resident_cover},
};

pub struct HeightEntry {
    pub generation: u64,
    pub page: HeightPage,
    last_used: u64,
}

#[derive(Clone, Copy)]
pub struct StreamingPolicy {
    pub selection: SelectionPolicy,
    pub height_cache: usize,
    pub material_cache: usize,
    pub draw_capacity: usize,
}
impl Default for StreamingPolicy {
    fn default() -> Self {
        Self {
            selection: SelectionPolicy::default(),
            height_cache: 1024,
            material_cache: 1024,
            draw_capacity: 1024,
        }
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct StreamUpdate {
    pub geometry_changed: bool,
    pub materials_changed: bool,
    pub selected: bool,
    pub height_ready: usize,
    pub material_ready: usize,
    pub capacity_rejected: bool,
}

pub struct TerrainStream {
    field: Arc<PlanetField>,
    policy: StreamingPolicy,
    history: SelectionHistory,
    fine: Vec<TileKey>,
    wanted: Vec<TileKey>,
    visible: Vec<TileKey>,
    heights: BTreeMap<TileKey, HeightEntry>,
    materials: MaterialPages,
    material_wanted: Vec<TileKey>,
    material_demand: Vec<u64>,
    height_jobs: BTreeMap<TileKey, JobHandle<Result<HeightPage, String>>>,
    material_jobs: BTreeMap<TileKey, JobHandle<CbtMaterialPage>>,
    epoch: u64,
    generation: u64,
}

impl TerrainStream {
    /// Synchronous six-root bootstrap guarantees a complete cover before the
    /// first frame. All later fine pages are built asynchronously without meshes.
    pub fn new(field: Arc<PlanetField>, policy: StreamingPolicy) -> Result<Self, String> {
        if policy.draw_capacity < 6
            || policy.height_cache < 6
            || policy.material_cache < 6
            || policy.selection.coarse_budget < 6
            || policy.selection.fine_budget < 6
            || policy.selection.coarse_level > 17
            || policy.selection.fine_level > 17
        {
            return Err("invalid terrain streaming budgets".into());
        }
        let visible: Vec<_> = (0..6).map(TileKey::root).collect();
        let mut heights = BTreeMap::new();
        let mut materials = MaterialPages::new();
        for &key in &visible {
            heights.insert(
                key,
                HeightEntry {
                    generation: 1,
                    page: lod::bake_height_page(&field, key, 33, 0.5).map_err(|e| e.to_string())?,
                    last_used: 0,
                },
            );
            materials.insert(
                lod::cbt_node_for_tile(key).unwrap().id(),
                (
                    1,
                    CbtMaterialPage::from_rgba8(lod::build_gpu_material_page(&field, key).rgba)
                        .ok_or("invalid material page")?,
                ),
            );
        }
        Ok(Self {
            field,
            policy,
            history: Default::default(),
            fine: vec![],
            wanted: vec![],
            visible,
            heights,
            materials,
            material_wanted: vec![],
            material_demand: (8..14).collect(),
            height_jobs: BTreeMap::new(),
            material_jobs: BTreeMap::new(),
            epoch: 0,
            generation: 1,
        })
    }

    pub fn visible(&self) -> &[TileKey] {
        &self.visible
    }
    pub fn radius_m(&self) -> f64 {
        self.field.params.radius_m
    }
    pub fn height(&self, key: TileKey) -> Option<&HeightEntry> {
        self.heights.get(&key)
    }
    pub fn materials(&self) -> &MaterialPages {
        &self.materials
    }
    pub fn material_demand(&self) -> &[u64] {
        &self.material_demand
    }
    pub fn pending_jobs(&self) -> (usize, usize) {
        (self.height_jobs.len(), self.material_jobs.len())
    }
    pub fn cache_entries(&self) -> (usize, usize) {
        (self.heights.len(), self.materials.len())
    }
    pub fn wanted_count(&self) -> usize {
        self.wanted.len()
    }

    fn next_generation(&mut self) -> Result<u64, String> {
        self.generation = self
            .generation
            .checked_add(1)
            .ok_or("terrain generation exhausted")?;
        Ok(self.generation)
    }

    pub fn tick(
        &mut self,
        view: SelectionView,
        pixel_angle: f64,
        now_s: f64,
    ) -> Result<StreamUpdate, String> {
        if !view.eye.is_finite()
            || !view.forward.is_finite()
            || view.forward.length_squared() < 1e-12
            || !view.stream_origin.is_finite()
            || !now_s.is_finite()
            || !pixel_angle.is_finite()
            || pixel_angle <= 0.0
            || view.frustum.is_some_and(|f| {
                !glam::DVec3::from(f.forward).is_finite()
                    || !f.cos_limit.is_finite()
                    || !(-1.0..=1.0).contains(&f.cos_limit)
            })
            || view.radius != self.field.params.radius_m
        {
            return Err("invalid terrain streaming view".into());
        }
        let mut update = StreamUpdate::default();
        if let Some(demand) = self.history.update(
            &self.field,
            view,
            now_s,
            &self.fine,
            self.wanted.is_empty(),
            true,
            self.policy.selection,
        ) {
            self.fine = demand.fine;
            self.wanted = demand.wanted;
            self.epoch = self.epoch.saturating_add(1);
            self.material_wanted =
                selection::material_view_selection(&self.fine, view, pixel_angle);
            update.selected = true;
        }
        let mut finished = Vec::new();
        for (&key, job) in &self.height_jobs {
            match job.try_take() {
                Ok(result) => finished.push((key, result)),
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    finished.push((key, Err("terrain height worker disconnected".into())))
                }
            }
        }
        for (key, result) in finished {
            self.height_jobs.remove(&key);
            let page = result?;
            let generation = self.next_generation()?;
            self.heights.insert(
                key,
                HeightEntry {
                    generation,
                    page,
                    last_used: self.epoch,
                },
            );
            update.height_ready += 1;
        }
        let mut finished = Vec::new();
        for (&key, job) in &self.material_jobs {
            match job.try_take() {
                Ok(page) => finished.push((key, Some(page))),
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => finished.push((key, None)),
            }
        }
        for (key, page) in finished {
            self.material_jobs.remove(&key);
            let page = page.ok_or("terrain material worker disconnected")?;
            // Material demand is independent of geometry LOD. Never discard a
            // useful finer page just because no height page shares its key.
            let generation = self.next_generation()?;
            self.materials.insert(
                lod::cbt_node_for_tile(key).ok_or("invalid tile")?.id(),
                (generation, page),
            );
            update.material_ready += 1;
            update.materials_changed = true;
        }
        // Camera-only frames poll the bounded mailboxes but do not rebuild
        // priorities, dependency sets or LRU staging vectors.
        if !update.selected && update.height_ready == 0 && update.material_ready == 0 {
            return Ok(update);
        }
        if (update.selected || update.height_ready > 0)
            && let Some(cover) = resident_cover(&self.wanted, &self.visible, |key| {
                self.heights.contains_key(&key)
            })
        {
            if cover.len() > self.policy.draw_capacity {
                update.capacity_rejected = true;
            } else if !cover.is_empty() && cover != self.visible {
                self.visible = cover;
                update.geometry_changed = true;
            }
        }
        let dependencies = selection::spawn_dependencies(&self.wanted, true, |key| {
            self.heights.contains_key(&key)
        });
        let protected: BTreeSet<_> = self
            .visible
            .iter()
            .copied()
            .chain(dependencies.iter().copied())
            .chain((0..6).map(TileKey::root))
            .chain(self.height_jobs.keys().copied())
            .collect();
        for key in &protected {
            if let Some(entry) = self.heights.get_mut(key) {
                entry.last_used = self.epoch;
            }
        }
        let entries: Vec<_> = self
            .heights
            .iter()
            .map(|(key, entry)| (*key, entry.last_used))
            .collect();
        // Reserve room for queued results too. Never evict the drawn cover.
        let reserve = self
            .policy
            .height_cache
            .saturating_sub(self.height_jobs.len() + 8);
        for key in lru_evictions(&entries, &protected, reserve) {
            self.heights.remove(&key);
        }
        let mut pending: Vec<_> = dependencies
            .iter()
            .copied()
            .filter(|key| !self.heights.contains_key(key) && !self.height_jobs.contains_key(key))
            .collect();
        pending.sort_by_cached_key(|key| selection::height_priority(&self.field, view, *key));
        let available = self
            .policy
            .height_cache
            .saturating_sub(self.heights.len() + self.height_jobs.len());
        update.capacity_rejected |= !pending.is_empty() && available == 0;
        for key in pending
            .into_iter()
            .take(8usize.saturating_sub(self.height_jobs.len()).min(available))
        {
            let field = self.field.clone();
            self.height_jobs.insert(
                key,
                JobHandle::spawn(move || {
                    lod::bake_height_page(&field, key, 33, 0.5).map_err(|e| e.to_string())
                }),
            );
        }
        let material_wanted = selection::material_demand(
            self.material_wanted
                .iter()
                .copied()
                .chain(self.visible.iter().copied())
                .chain(
                    self.wanted
                        .iter()
                        .copied()
                        .filter(|key| self.heights.contains_key(key)),
                ),
            view,
        );
        let ids: Vec<_> = material_wanted
            .iter()
            .map(|key| lod::cbt_node_for_tile(*key).unwrap().id())
            .collect();
        if ids != self.material_demand {
            self.material_demand = ids;
            update.materials_changed = true;
        }
        for key in material_wanted {
            let key = selection::material_build_dependency(key, |key| {
                self.materials
                    .contains_key(&lod::cbt_node_for_tile(key).unwrap().id())
            });
            let budget = if self.height_jobs.len() < 4 { 4 } else { 2 };
            if self.material_jobs.len() >= budget {
                break;
            }
            if self
                .materials
                .contains_key(&lod::cbt_node_for_tile(key).unwrap().id())
                || self.material_jobs.contains_key(&key)
            {
                continue;
            }
            self.material_jobs
                .insert(key, crate::spawn_material_job(self.field.clone(), key));
        }
        let pinned: BTreeSet<_> =
            resident_material_hierarchy(self.material_demand.iter().copied(), |id| {
                self.materials.contains_key(&id)
            })
            .into_iter()
            .take(self.policy.material_cache)
            .collect();
        let mut cold: Vec<_> = self
            .materials
            .iter()
            .filter(|(id, _)| !pinned.contains(id))
            .map(|(id, (generation, _))| (*generation, *id))
            .collect();
        cold.sort_unstable();
        for (_, id) in cold.into_iter().take(
            self.materials
                .len()
                .saturating_sub(self.policy.material_cache),
        ) {
            self.materials.remove(&id);
            update.materials_changed = true;
        }
        Ok(update)
    }
}
