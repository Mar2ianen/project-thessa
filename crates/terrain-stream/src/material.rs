//! Game-specific material job/cache lifecycle, shared by native and Bevy hosts.
//! Height residency and renderer publication never own or invalidate this state.
#[cfg(test)]
#[path = "material_tests.rs"]
mod tests;
use crate::{
    JobHandle,
    selection::{self, SelectionView},
};
use std::{
    collections::BTreeMap,
    sync::{Arc, mpsc::TryRecvError},
};
use thessa_rcbt_core::{material_page::CbtMaterialPage, material_source::MaterialPageStore};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::{self, TileKey},
};

#[derive(Default, Debug, Clone, Copy)]
pub struct MaterialPoll {
    pub ready: usize,
    pub failed: usize,
}

#[derive(Clone, Copy)]
pub struct MaterialDemand<'a> {
    /// Host increments when its accepted/debounced view or fine selection changes.
    pub selection_epoch: u64,
    pub fine: &'a [TileKey],
    pub view: SelectionView,
    pub pixel_angle: f64,
    pub height_jobs: usize,
}

pub struct MaterialStream {
    pub(crate) source: MaterialPageStore,
    pub(crate) jobs: BTreeMap<TileKey, JobHandle<CbtMaterialPage>>,
    refined: Vec<TileKey>,
    refinement: Option<(u64, u64)>,
    cache_budget: usize,
    normal_maps: bool,
    ground_detail: bool,
    ground_relief: bool,
    max_level: u8,
    reference_height_m: f64,
}

impl MaterialStream {
    pub fn new(cache_budget: usize) -> Self {
        Self::with_source(cache_budget, MaterialPageStore::default())
    }

    pub fn with_source(cache_budget: usize, source: MaterialPageStore) -> Self {
        Self::with_source_and_normals(cache_budget, source, false)
    }

    pub fn with_source_and_normals(
        cache_budget: usize,
        source: MaterialPageStore,
        normal_maps: bool,
    ) -> Self {
        Self::with_detail(cache_budget, source, normal_maps, false, 17)
    }

    /// Immutable producer policy: restart/invalidate the stream when changing
    /// it, so old pages cannot masquerade as the new optical representation.
    pub fn with_detail(
        cache_budget: usize,
        source: MaterialPageStore,
        normal_maps: bool,
        ground_detail: bool,
        max_level: u8,
    ) -> Self {
        Self::with_relief(
            cache_budget,
            source,
            normal_maps,
            ground_detail,
            max_level,
            false,
        )
    }
    pub fn with_relief(
        cache_budget: usize,
        mut source: MaterialPageStore,
        normal_maps: bool,
        ground_detail: bool,
        max_level: u8,
        ground_relief: bool,
    ) -> Self {
        source.trim_to_budget(cache_budget);
        Self {
            source,
            jobs: BTreeMap::new(),
            refined: Vec::new(),
            refinement: None,
            cache_budget,
            normal_maps,
            ground_detail,
            ground_relief,
            max_level: if ground_detail {
                max_level.clamp(17, 24)
            } else {
                17
            },
            reference_height_m: 0.0,
        }
    }

    pub fn source(&self) -> &MaterialPageStore {
        &self.source
    }
    pub(crate) fn refined(&self) -> &[TileKey] {
        &self.refined
    }
    pub fn pending_count(&self) -> usize {
        self.jobs.len()
    }
    /// Cached optical selection reference, not authoritative geometry elevation.
    pub fn reference_height_m(&self) -> f64 {
        self.reference_height_m
    }
    pub fn needs_refinement(&self, epoch: u64, pixel_angle: f64) -> bool {
        self.refinement != Some((epoch, pixel_angle.to_bits()))
    }

    /// Nonblocking bounded mailbox polling. Disconnected workers release their
    /// slots and are reported, so the next reconcile can retry rather than wedge.
    /// Up to four results can transiently exceed the source budget until reconcile.
    pub fn poll(&mut self) -> MaterialPoll {
        let mut completed = Vec::new();
        for (&key, job) in &self.jobs {
            match job.try_take() {
                Ok(page) => completed.push((key, Some(page))),
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => completed.push((key, None)),
            }
        }
        let mut result = MaterialPoll::default();
        let mut ready = Vec::new();
        for (key, page) in completed {
            self.jobs.remove(&key);
            if let Some(page) = page {
                if let Some(node) = lod::cbt_node_for_tile(key) {
                    ready.push((node.id(), page));
                    result.ready += 1;
                } else {
                    result.failed += 1;
                }
            } else {
                result.failed += 1;
            }
        }
        self.source.set_pages(ready);
        result
    }

    pub fn reconcile(
        &mut self,
        field: &Arc<PlanetField>,
        demand: MaterialDemand<'_>,
        geometry_sources: impl IntoIterator<Item = TileKey>,
    ) {
        if self.ground_detail && self.needs_refinement(demand.selection_epoch, demand.pixel_angle) {
            // One canonical reference per selection epoch, not per page/tick.
            // Treating elevated ground as datum made a standing camera appear
            // hundreds of metres from highlands and starved fine material LOD.
            let direction = demand.view.eye.normalize_or_zero();
            self.reference_height_m = if direction != glam::DVec3::ZERO {
                field.height_m(direction.to_array(), 32.0)
            } else {
                0.0
            };
        }
        let normal_maps = self.normal_maps;
        let ground_detail = self.ground_detail;
        let ground_relief = self.ground_relief;
        self.reconcile_with(demand, geometry_sources, |key| {
            crate::spawn_material_job_with_relief(
                field.clone(),
                key,
                normal_maps,
                ground_detail,
                ground_relief,
            )
        });
    }

    fn reconcile_with(
        &mut self,
        demand: MaterialDemand<'_>,
        geometry_sources: impl IntoIterator<Item = TileKey>,
        mut spawn: impl FnMut(TileKey) -> JobHandle<CbtMaterialPage>,
    ) {
        if self.needs_refinement(demand.selection_epoch, demand.pixel_angle) {
            self.refined = selection::material_view_selection_at_height(
                demand.fine,
                demand.view,
                demand.pixel_angle,
                self.max_level,
                self.reference_height_m,
            );
            self.refinement = Some((demand.selection_epoch, demand.pixel_angle.to_bits()));
        }
        let wanted = selection::material_demand_at_height(
            self.refined.iter().copied().chain(geometry_sources),
            demand.view,
            self.reference_height_m,
        );
        let ids = wanted
            .iter()
            .filter_map(|&key| lod::cbt_node_for_tile(key).map(|node| node.id()))
            .collect();
        self.source.set_priority(ids);
        self.source.trim_to_budget(self.cache_budget);
        if self.cache_budget == 0 {
            return;
        }
        let budget = if demand.height_jobs < 4 { 4 } else { 2 };
        for key in wanted {
            if self.jobs.len() >= budget {
                break;
            }
            let key = selection::material_build_dependency(key, |key| {
                lod::cbt_node_for_tile(key).is_some_and(|node| self.source.contains_page(node.id()))
            });
            if lod::cbt_node_for_tile(key).is_none_or(|node| self.source.contains_page(node.id()))
                || self.jobs.contains_key(&key)
            {
                continue;
            }
            self.jobs.insert(key, spawn(key));
        }
    }
}
