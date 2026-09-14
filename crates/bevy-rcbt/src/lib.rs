//! Thin Bevy-side ownership and scheduling adapter for RCBT.
//!
//! This crate intentionally does not define topology semantics or hold wgpu
//! handles. A later render-world implementation can consume this resource and
//! submit work through `thessa-rcbt-wgpu` without changing `rcbt-core`.

use bevy::prelude::{App, Plugin, Resource};
use thessa_rcbt_core::{CbtCapabilities, FrameBudget, LeafCandidate, Tree, UpdatePlan, plan_frame};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RenderView {
    pub eye_body_m: [f64; 3],
    pub forward_body: [f64; 3],
    pub velocity_body_mps: [f64; 3],
    pub fov_rad: f64,
    pub pixel_error_target: f32,
}

#[derive(Resource)]
pub struct CbtRenderState {
    topology: Tree,
    view: Option<RenderView>,
    capabilities: CbtCapabilities,
}

impl CbtRenderState {
    pub fn new(max_depth: u8) -> Result<Self, thessa_rcbt_core::TreeError> {
        Ok(Self {
            topology: Tree::new(max_depth)?,
            view: None,
            capabilities: CbtCapabilities::default(),
        })
    }

    pub fn topology(&self) -> &Tree {
        &self.topology
    }

    pub fn view(&self) -> Option<RenderView> {
        self.view
    }

    pub fn capabilities(&self) -> CbtCapabilities {
        self.capabilities
    }

    pub fn set_capabilities(&mut self, capabilities: CbtCapabilities) {
        self.capabilities = capabilities;
    }

    pub fn set_view(&mut self, view: RenderView) {
        self.view = Some(view);
    }

    pub fn plan_frame<I>(&self, candidates: I, budget: FrameBudget) -> UpdatePlan
    where
        I: IntoIterator<Item = LeafCandidate>,
    {
        plan_frame(&self.topology, candidates, budget)
    }

    pub fn commit(&mut self, plan: &UpdatePlan) -> Result<(), thessa_rcbt_core::TreeError> {
        self.topology.apply_batch(plan.updates())
    }
}

pub struct CbtPlugin {
    pub max_depth: u8,
}

impl Plugin for CbtPlugin {
    fn build(&self, app: &mut App) {
        let state = CbtRenderState::new(self.max_depth).expect("valid RCBT plugin depth");
        app.insert_resource(state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_rcbt_core::{CandidateAction, WorkClass};

    #[test]
    fn plugin_owns_client_state_without_changing_core_semantics() {
        let mut app = App::new();
        app.add_plugins(CbtPlugin { max_depth: 8 });
        let candidates = [LeafCandidate {
            node: thessa_rcbt_core::Node::root(),
            action: CandidateAction::Split,
            class: WorkClass::CoverageRepair,
            projected_error_px: 2.0,
            predicted_error_px: 0.0,
            time_to_needed_s: 1.0,
        }];
        let plan = app
            .world()
            .resource::<CbtRenderState>()
            .plan_frame(candidates, FrameBudget { max_operations: 1 });
        app.world_mut()
            .resource_mut::<CbtRenderState>()
            .commit(&plan)
            .unwrap();
        assert_eq!(
            app.world()
                .resource::<CbtRenderState>()
                .topology()
                .leaf_count(),
            2
        );
    }
}
