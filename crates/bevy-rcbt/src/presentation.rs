//! Disposable Bevy bridge acknowledgement: CPU assets alone are not drawable.
use crate::{CbtRenderMaterial, CbtRenderSurface};
use bevy::{
    asset::AssetId,
    prelude::{Image, Resource},
};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DrawStamp {
    epoch: u64,
    surface: u64,
    mesh: bool,
    albedo: Option<AssetId<Image>>,
    roughness: Option<AssetId<Image>>,
}

impl DrawStamp {
    fn new(surface: &CbtRenderSurface, material: Option<&CbtRenderMaterial>) -> Self {
        Self {
            epoch: surface.presentation_epoch,
            surface: surface.generation(),
            mesh: surface.gpu_mesh_enabled(),
            albedo: material.map(|m| m.albedo.id()),
            roughness: material.and_then(|m| m.roughness.as_ref().map(|h| h.id())),
        }
    }
}

/// Shared across extraction. Acknowledges a submitted draw with GPU material
/// bindings, never just a CPU Image. Geometry generations are intentionally
/// excluded: complete snapshots replace each other within one GPU submission.
#[derive(Debug, Default, Clone, Resource)]
pub struct CbtGpuPresentation(Arc<Mutex<Option<DrawStamp>>>);

impl CbtGpuPresentation {
    pub fn is_ready(
        &self,
        surface: &CbtRenderSurface,
        material: Option<&CbtRenderMaterial>,
    ) -> bool {
        surface.gpu_surface_ready()
            && surface.gpu_raster_enabled()
            && *self.0.lock().unwrap() == Some(DrawStamp::new(surface, material))
    }

    pub(crate) fn attempt(
        &self,
        surface: &CbtRenderSurface,
        material: Option<&CbtRenderMaterial>,
    ) -> DrawAttempt<'_> {
        DrawAttempt {
            ack: self,
            stamp: DrawStamp::new(surface, material),
            submitted: false,
        }
    }
}

pub(crate) struct DrawAttempt<'a> {
    ack: &'a CbtGpuPresentation,
    stamp: DrawStamp,
    pub submitted: bool,
}

impl Drop for DrawAttempt<'_> {
    fn drop(&mut self) {
        // Publish once at the end; clearing at entry races the next main frame.
        *self.ack.0.lock().unwrap() = self.submitted.then_some(self.stamp);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bootstrap_waits_for_draw_and_rearms_on_activation_or_material_change() {
        let ack = CbtGpuPresentation::default();
        let render_ack = ack.clone();
        let mut surface = CbtRenderSurface::new(3_200_000.0);
        let mut material = CbtRenderMaterial::default();
        surface.set_gpu_raster_enabled(true);
        surface.set_gpu_surface_ready(true);
        assert!(!ack.is_ready(&surface, Some(&material)));
        // CPU material exists but a render-world GPU binding is still absent.
        drop(render_ack.attempt(&surface, Some(&material)));
        assert!(!ack.is_ready(&surface, Some(&material)));
        render_ack.attempt(&surface, Some(&material)).submitted = true;
        assert!(ack.is_ready(&surface, Some(&material)));
        surface.set_gpu_surface_ready(false);
        surface.set_gpu_surface_ready(true);
        assert!(!ack.is_ready(&surface, Some(&material)));
        render_ack.attempt(&surface, Some(&material)).submitted = true;
        material.roughness = Some(Default::default());
        assert!(!ack.is_ready(&surface, Some(&material)));
        render_ack.attempt(&surface, Some(&material)).submitted = true;
        assert!(ack.is_ready(&surface, Some(&material)));
        drop(render_ack.attempt(&surface, Some(&material)));
        assert!(!ack.is_ready(&surface, Some(&material)));
    }
}
