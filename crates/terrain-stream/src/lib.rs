//! Game-specific streaming state/policy shared by native and Bevy hosts.
//! Extracted game code stays GPL; reusable MIT renderer crates never depend on it.
#![forbid(unsafe_code)]
pub mod height;
pub mod material;
pub mod runtime;
pub mod selection;
#[cfg(test)]
mod tests;

use std::sync::{
    Mutex,
    mpsc::{Receiver, TryRecvError},
};

/// Existing capacity-one worker mailbox, shared with the CPU-mesh bridge too.
/// Sync only for host resource requirements; producers never lock this mutex.
pub struct JobHandle<T> {
    rx: Mutex<Receiver<T>>,
}

impl<T> JobHandle<T> {
    /// The same bounded Rayon executor for native and ECS hosts. Dropping a
    /// receiver discards late output; it never blocks a worker on frame work.
    pub fn spawn(build: impl FnOnce() -> T + Send + 'static) -> Self
    where
        T: Send + 'static,
    {
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        rayon::spawn(move || {
            let _ = tx.send(build());
        });
        Self::new(rx)
    }
    pub fn new(rx: Receiver<T>) -> Self {
        Self { rx: Mutex::new(rx) }
    }
    pub fn try_take(&self) -> Result<T, TryRecvError> {
        self.rx.lock().expect("terrain job mailbox").try_recv()
    }
}

pub fn spawn_material_job(
    field: std::sync::Arc<thessa_worldgen_rocky::field::PlanetField>,
    key: thessa_worldgen_rocky::lod::TileKey,
) -> JobHandle<thessa_rcbt_core::material_page::CbtMaterialPage> {
    spawn_material_job_with_normals(field, key, false)
}

pub fn spawn_material_job_with_normals(
    field: std::sync::Arc<thessa_worldgen_rocky::field::PlanetField>,
    key: thessa_worldgen_rocky::lod::TileKey,
    normal_maps: bool,
) -> JobHandle<thessa_rcbt_core::material_page::CbtMaterialPage> {
    spawn_material_job_with_detail(field, key, normal_maps, false)
}

pub fn spawn_material_job_with_detail(
    field: std::sync::Arc<thessa_worldgen_rocky::field::PlanetField>,
    key: thessa_worldgen_rocky::lod::TileKey,
    normal_maps: bool,
    ground_detail: bool,
) -> JobHandle<thessa_rcbt_core::material_page::CbtMaterialPage> {
    JobHandle::spawn(move || {
        let page = thessa_worldgen_rocky::lod::build_gpu_material_page_with_ground_detail(
            &field,
            key,
            normal_maps,
            ground_detail,
        );
        thessa_rcbt_core::material_page::CbtMaterialPage::from_rgba8_with_normals(
            page.rgba,
            normal_maps.then_some(page.normal),
        )
        .expect("fixed material page layout")
    })
}
