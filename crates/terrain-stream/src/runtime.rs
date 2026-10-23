//! Renderer-free execution of the extracted terrain policy on the Rayon pool.
//! Hosts publish snapshots only when coverage/material state changes.
#[cfg(test)]
#[path = "runtime_tests.rs"]
mod tests;
use crate::{
    height::{self, HeightBuild, HeightEntry, HeightPolicy, HeightStream},
    material::{MaterialDemand, MaterialStream},
    selection::{SelectionHistory, SelectionPolicy, SelectionView},
};
use std::sync::Arc;
use thessa_rcbt_core::{
    material_page::{CbtMaterialPage, MaterialPages},
    material_source::MaterialPageStore,
};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::{self, TileKey},
    streaming::resident_cover,
};

#[derive(Clone, Copy)]
pub struct StreamingPolicy {
    pub selection: SelectionPolicy,
    pub height_cache: usize,
    pub material_cache: usize,
    pub draw_capacity: usize,
    pub normal_maps: bool,
    pub ground_detail: bool,
    pub material_max_level: u8,
}
impl Default for StreamingPolicy {
    fn default() -> Self {
        Self {
            selection: SelectionPolicy::default(),
            height_cache: 1024,
            material_cache: 1024,
            draw_capacity: 1024,
            normal_maps: true,
            ground_detail: false,
            material_max_level: 17,
        }
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct StreamUpdate {
    pub geometry_changed: bool,
    pub materials_changed: bool,
    pub selected: bool,
    pub height_ready: usize,
    pub height_failed: usize,
    pub material_ready: usize,
    pub material_failed: usize,
    pub capacity_rejected: bool,
}

pub struct TerrainStream {
    field: Arc<PlanetField>,
    policy: StreamingPolicy,
    history: SelectionHistory,
    fine: Vec<TileKey>,
    wanted: Vec<TileKey>,
    visible: Vec<TileKey>,
    heights: HeightStream,
    materials: MaterialStream,
    epoch: u64,
}

impl TerrainStream {
    /// Immutable canonical source shared with physics/other host consumers;
    /// taking an Arc clone does not duplicate terrain or transfer cache policy.
    pub fn field(&self) -> &Arc<PlanetField> {
        &self.field
    }
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
        let mut heights = HeightStream::default();
        let mut materials = MaterialPageStore::default();
        for &key in &visible {
            heights.insert_batch(
                0,
                [(
                    key,
                    HeightBuild {
                        page: height::bake_page(&field, key)?,
                        payload: (),
                    },
                )],
                |_, ()| (),
            );
            let page = lod::build_gpu_material_page_with_ground_detail(
                &field,
                key,
                policy.normal_maps,
                policy.ground_detail,
            );
            materials.set_page(
                lod::cbt_node_for_tile(key).unwrap().id(),
                CbtMaterialPage::from_rgba8_with_normals(
                    page.rgba,
                    policy.normal_maps.then_some(page.normal),
                )
                .ok_or("invalid material page")?,
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
            materials: {
                materials.set_priority((8..14).collect());
                MaterialStream::with_detail(
                    policy.material_cache,
                    materials,
                    policy.normal_maps,
                    policy.ground_detail,
                    policy.material_max_level,
                )
            },
            epoch: 0,
        })
    }

    pub fn visible(&self) -> &[TileKey] {
        &self.visible
    }
    pub fn radius_m(&self) -> f64 {
        self.field.params.radius_m
    }
    pub fn height(&self, key: TileKey) -> Option<HeightEntry<'_>> {
        self.heights.height(key)
    }
    pub fn materials(&self) -> &MaterialPages {
        self.materials.source().source_pages()
    }
    pub fn material_demand(&self) -> &[u64] {
        self.materials.source().priority()
    }
    pub fn material_reference_height_m(&self) -> f64 {
        self.materials.reference_height_m()
    }
    pub fn pending_jobs(&self) -> (usize, usize) {
        (self.heights.pending_count(), self.materials.pending_count())
    }
    pub fn cache_entries(&self) -> (usize, usize) {
        (
            self.heights.len(),
            self.materials.source().source_pages().len(),
        )
    }
    pub fn wanted_count(&self) -> usize {
        self.wanted.len()
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
            update.selected = true;
        }
        let height_poll = self.heights.poll_with(self.epoch, |_, ()| ());
        update.height_ready = height_poll.ready;
        update.height_failed = height_poll.failed.len();
        let material_revision = self.materials.source().generation();
        let material_poll = self.materials.poll();
        update.material_ready = material_poll.ready;
        update.material_failed = material_poll.failed;
        // Camera-only frames poll the bounded mailboxes but do not rebuild
        // priorities, dependency sets or LRU staging vectors.
        if !update.selected
            && update.height_ready == 0
            && update.height_failed == 0
            && update.material_ready == 0
            && update.material_failed == 0
            && !self.materials.needs_refinement(self.epoch, pixel_angle)
        {
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
        update.geometry_changed |= height_poll
            .changed
            .iter()
            .any(|key| self.visible.contains(key));
        let admission = self.heights.reconcile(
            &self.field,
            view,
            &self.wanted,
            self.visible.iter().copied(),
            self.epoch,
            HeightPolicy {
                capacity: self.policy.height_cache,
                max_jobs: 8,
                ancestors: true,
                pin_roots: true,
            },
            |key| height::spawn_height_job(self.field.clone(), key),
        );
        update.capacity_rejected |= admission.capacity_rejected;
        self.materials.reconcile(
            &self.field,
            MaterialDemand {
                selection_epoch: self.epoch,
                fine: &self.fine,
                view,
                pixel_angle,
                height_jobs: self.heights.pending_count(),
            },
            self.visible.iter().copied().chain(
                self.wanted
                    .iter()
                    .copied()
                    .filter(|key| self.heights.contains_key(key)),
            ),
        );
        update.materials_changed = self.materials.source().generation() != material_revision;
        Ok(update)
    }
}
