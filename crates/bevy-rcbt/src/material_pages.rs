//! Thin Bevy resource/extraction bridge over backend-neutral material sources.
use bevy::prelude::{Deref, DerefMut, Resource};
pub use thessa_rcbt_core::material_page::{CbtMaterialPage, MATERIAL_PAGE_SIZE};
use thessa_rcbt_core::material_source::MaterialPageStore;

#[derive(Debug, Clone, Default, Resource, Deref, DerefMut)]
pub struct CbtRenderMaterialPages(pub MaterialPageStore);

impl CbtRenderMaterialPages {
    /// Called only for dirty source revisions. Directory and immutable mip
    /// payloads are shared with the application owner, never copied for Bevy.
    /// Hosts must check revision through a shared borrow before invoking this
    /// through ResMut, to avoid spurious Bevy extraction/change detection.
    pub fn publish_source(&mut self, source: &MaterialPageStore) {
        self.0 = source.clone();
    }

    /// Shared revision check must precede dereferencing ResMut mutably.
    pub fn publish_if_changed(
        source: &MaterialPageStore,
        resource: &mut bevy::prelude::ResMut<'_, Self>,
    ) {
        if resource.generation() != source.generation() {
            resource.publish_source(source);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bridge_shares_source_and_old_snapshot_survives_new_publication() {
        let mut source = MaterialPageStore::default();
        source.set_page(
            8,
            CbtMaterialPage::from_rgba8(vec![32; 128 * 128 * 4]).unwrap(),
        );
        let mut bridge = CbtRenderMaterialPages::default();
        bridge.publish_source(&source);
        let old = bridge.clone();
        assert!(std::ptr::eq(source.source_pages(), bridge.source_pages()));
        source.remove_page(8);
        bridge.publish_source(&source);
        assert!(bridge.source_pages().is_empty());
        assert!(old.contains_page(8));
    }

    #[test]
    fn unchanged_publication_does_not_set_bevy_change_flag() {
        use bevy::prelude::{DetectChanges, Res, ResMut, Schedule, World};
        #[derive(Resource)]
        struct Source(MaterialPageStore);
        fn publish(source: Res<Source>, mut resource: ResMut<CbtRenderMaterialPages>) {
            CbtRenderMaterialPages::publish_if_changed(&source.0, &mut resource);
        }
        let mut world = World::new();
        world.init_resource::<CbtRenderMaterialPages>();
        let mut source = MaterialPageStore::default();
        source.set_page(
            8,
            CbtMaterialPage::from_rgba8(vec![32; 128 * 128 * 4]).unwrap(),
        );
        world.insert_resource(Source(source));
        let mut schedule = Schedule::default();
        schedule.add_systems(publish);
        schedule.run(&mut world);
        assert!(world.resource_ref::<CbtRenderMaterialPages>().is_changed());
        world.clear_trackers();
        schedule.run(&mut world);
        assert!(!world.resource_ref::<CbtRenderMaterialPages>().is_changed());
        world.resource_mut::<Source>().0.set_priority(vec![8]);
        schedule.run(&mut world);
        assert!(world.resource_ref::<CbtRenderMaterialPages>().is_changed());
    }
}
