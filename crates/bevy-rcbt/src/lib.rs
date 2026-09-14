//! Universal Bevy scheduling adapter for the backend-neutral RCBT contracts.
//!
//! The plugin owns frame state and bounded topology commits only. It does not
//! know about cube-sphere addresses, terrain materials, or meshes. With the
//! optional `render` feature it also extracts an exact depth-agnostic leaf
//! stream into Bevy's render world; the render adapter owns page, vertex, and
//! indirect-draw buffers and no authoritative state crosses that boundary.

use bevy::prelude::{App, Plugin, PostUpdate, ResMut, Resource};
use std::collections::BTreeMap;
use thessa_rcbt_core::{
    CbtCapabilities, FrameBudget, HeightPage, LeafCandidate, LeafList, Tree, TreeError, Update,
    UpdatePlan, plan_frame,
};

/// Exact render transport record for one committed CBT leaf.
///
/// The node id is a `u64` split into two `u32`s because portable WGSL does
/// not require storage `u64`. `depth` and `ordinal` are explicit, so a GPU
/// consumer never has to infer precision-sensitive data from buffer layout.
pub type CbtLeafRecord = [u32; 4];

/// Main-world snapshot consumed by the optional render-world integration.
///
/// This is deliberately a leaf stream rather than a dense bitfield: the game
/// uses a depth-37 cube-sphere CBT, while the dense compact layout is capped at
/// depth 20. The stream is exact, bounded by the active leaf count, and only
/// rebuilt after a topology commit.
#[derive(Debug, Clone, Default, Resource)]
pub struct CbtRenderTopology {
    generation: u64,
    max_depth: u8,
    records: Vec<CbtLeafRecord>,
}

impl CbtRenderTopology {
    fn from_tree(tree: &Tree) -> Self {
        Self::from_leaf_list(0, tree.max_depth(), &tree.snapshot())
    }

    fn from_leaf_list(generation: u64, max_depth: u8, leaves: &LeafList) -> Self {
        let records = leaves
            .iter()
            .enumerate()
            .map(|(ordinal, node)| {
                [
                    node.id() as u32,
                    (node.id() >> 32) as u32,
                    u32::from(node.depth()),
                    ordinal as u32,
                ]
            })
            .collect();
        Self {
            generation,
            max_depth,
            records,
        }
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn max_depth(&self) -> u8 {
        self.max_depth
    }

    pub fn records(&self) -> &[CbtLeafRecord] {
        &self.records
    }

    pub fn leaf_count(&self) -> usize {
        self.records.len()
    }
}

/// Immutable height pages supplied by the terrain adapter for the current
/// CBT leaves. Pages are keyed by the exact heap node id, not by a dense tree
/// slot, so odd-depth transport leaves and depth-37 game tiles remain
/// representable without a second topology walk.
#[derive(Debug, Clone, Default, Resource)]
pub struct CbtRenderPages {
    generation: u64,
    pages: BTreeMap<u64, HeightPage>,
}

impl CbtRenderPages {
    /// Insert or replace the baked page for one CBT leaf.
    pub fn set_page(&mut self, node_id: u64, page: HeightPage) {
        if self.pages.get(&node_id) != Some(&page) {
            self.pages.insert(node_id, page);
            self.generation = self.generation.saturating_add(1);
        }
    }

    /// Remove a page whose CPU-side tile cache was evicted.
    pub fn remove_page(&mut self, node_id: u64) {
        if self.pages.remove(&node_id).is_some() {
            self.generation = self.generation.saturating_add(1);
        }
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn len(&self) -> usize {
        self.pages.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pages.is_empty()
    }

    #[cfg(feature = "render")]
    pub(crate) fn get(&self, node_id: u64) -> Option<&HeightPage> {
        self.pages.get(&node_id)
    }
}

/// Surface constants consumed by the GPU page-to-vertex expansion. The
/// authoritative field and CPU surface queries remain outside this resource;
/// this is only render geometry metadata.
#[derive(Debug, Clone, Copy, Resource)]
pub struct CbtRenderSurface {
    radius_m: f32,
    generation: u64,
    render_from_body: [f32; 16],
    transform_generation: u64,
    gpu_raster_enabled: bool,
}

impl Default for CbtRenderSurface {
    fn default() -> Self {
        Self {
            radius_m: 1.0,
            generation: 0,
            render_from_body: [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
            transform_generation: 0,
            gpu_raster_enabled: false,
        }
    }
}

impl CbtRenderSurface {
    pub fn new(radius_m: f32) -> Self {
        let mut surface = Self::default();
        surface.set_radius_m(radius_m);
        surface
    }

    pub fn radius_m(&self) -> f32 {
        self.radius_m
    }

    pub fn set_radius_m(&mut self, radius_m: f32) {
        if radius_m.is_finite() && radius_m > 0.0 && self.radius_m != radius_m {
            self.radius_m = radius_m;
            self.generation = self.generation.saturating_add(1);
        }
    }

    /// Set the body-space to Bevy render-local affine transform used by the
    /// optional GPU raster consumer. The matrix is column-major, matching
    /// `Mat4::to_cols_array`.
    pub fn set_render_from_body(&mut self, matrix: [f32; 16]) {
        if matrix.iter().all(|value| value.is_finite()) && self.render_from_body != matrix {
            self.render_from_body = matrix;
            self.transform_generation = self.transform_generation.saturating_add(1);
        }
    }

    pub fn gpu_raster_enabled(&self) -> bool {
        self.gpu_raster_enabled
    }

    /// Opt into the experimental direct raster path. It is disabled by
    /// default while the CPU mesh fallback remains the authoritative visible
    /// cover; callers should enable it only after their draw-list policy
    /// avoids drawing the same cover twice.
    pub fn set_gpu_raster_enabled(&mut self, enabled: bool) {
        self.gpu_raster_enabled = enabled;
    }

    #[cfg(feature = "render")]
    pub(crate) fn generation(&self) -> u64 {
        self.generation
    }

    #[cfg(feature = "render")]
    pub(crate) fn render_from_body(&self) -> [f32; 16] {
        self.render_from_body
    }

    #[cfg(feature = "render")]
    pub(crate) fn transform_generation(&self) -> u64 {
        self.transform_generation
    }
}

/// Camera state supplied by a domain adapter in its own coordinate frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RenderView {
    pub eye_body_m: [f64; 3],
    pub forward_body: [f64; 3],
    pub velocity_body_mps: [f64; 3],
    pub fov_rad: f64,
    pub pixel_error_target: f32,
}

/// Input submitted for one render-frame topology decision.
#[derive(Debug, Default, Resource)]
pub struct CbtFrameInput {
    view: Option<RenderView>,
    candidates: Vec<LeafCandidate>,
    budget: Option<FrameBudget>,
    pending: bool,
}

impl CbtFrameInput {
    /// Replace the pending candidate set. The plugin consumes it once during
    /// `PostUpdate`; stale work is never replayed on a later frame.
    pub fn submit<I>(&mut self, view: Option<RenderView>, candidates: I)
    where
        I: IntoIterator<Item = LeafCandidate>,
    {
        self.view = view;
        self.candidates.clear();
        self.candidates.extend(candidates);
        self.pending = true;
    }

    pub fn set_budget(&mut self, budget: Option<FrameBudget>) {
        self.budget = budget;
    }

    fn take(&mut self) -> Option<(Option<RenderView>, Vec<LeafCandidate>, Option<FrameBudget>)> {
        self.pending.then(|| {
            self.pending = false;
            (self.view, std::mem::take(&mut self.candidates), self.budget)
        })
    }
}

/// The authoritative client-side CBT state. The tree is render topology, not
/// physical terrain state; authoritative surface queries remain elsewhere.
#[derive(Resource)]
pub struct CbtRenderState {
    topology: Tree,
    view: Option<RenderView>,
    capabilities: CbtCapabilities,
    frame_budget: FrameBudget,
}

impl CbtRenderState {
    pub fn new(max_depth: u8) -> Result<Self, TreeError> {
        Self::with_budget(max_depth, FrameBudget::default())
    }

    pub fn with_budget(max_depth: u8, frame_budget: FrameBudget) -> Result<Self, TreeError> {
        Self::with_initial_depth(max_depth, 0, frame_budget)
    }

    pub fn with_initial_depth(
        max_depth: u8,
        initial_depth: u8,
        frame_budget: FrameBudget,
    ) -> Result<Self, TreeError> {
        Ok(Self {
            topology: Tree::at_depth(max_depth, initial_depth)?,
            view: None,
            capabilities: CbtCapabilities::default(),
            frame_budget,
        })
    }

    pub fn topology(&self) -> &Tree {
        &self.topology
    }

    pub fn leaf_list(&self) -> LeafList {
        self.topology.snapshot()
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

    pub fn frame_budget(&self) -> FrameBudget {
        self.frame_budget
    }

    pub fn set_frame_budget(&mut self, frame_budget: FrameBudget) {
        self.frame_budget = frame_budget;
    }

    pub fn plan_frame<I>(&self, candidates: I, budget: FrameBudget) -> UpdatePlan
    where
        I: IntoIterator<Item = LeafCandidate>,
    {
        plan_frame(&self.topology, candidates, budget)
    }

    pub fn commit(&mut self, plan: &UpdatePlan) -> Result<(), TreeError> {
        self.topology.apply_batch(plan.updates())
    }
}

/// Result of the last consumed frame input. A renderer/domain adapter can use
/// `updates` to upload only changed ranges and `leaf_list` to build a draw list.
#[derive(Debug, Default, Resource)]
pub struct CbtFrameOutput {
    pub frame_index: u64,
    pub topology_generation: u64,
    pub updates: Vec<Update>,
    pub leaf_list: LeafList,
    pub error: Option<String>,
}

/// Backend-neutral Bevy plugin. `max_depth` is a topology contract; the
/// plugin never interprets it as terrain LOD or as a physical resolution.
#[derive(Debug, Clone, Copy)]
pub struct CbtPlugin {
    pub max_depth: u8,
    pub initial_depth: u8,
    pub frame_budget: FrameBudget,
}

impl Default for CbtPlugin {
    fn default() -> Self {
        Self {
            max_depth: 16,
            initial_depth: 0,
            frame_budget: FrameBudget::default(),
        }
    }
}

impl Plugin for CbtPlugin {
    fn build(&self, app: &mut App) {
        let state = CbtRenderState::with_initial_depth(
            self.max_depth,
            self.initial_depth,
            self.frame_budget,
        )
        .expect("valid RCBT plugin depth");
        let topology = CbtRenderTopology::from_tree(&state.topology);
        app.insert_resource(state)
            .insert_resource(topology)
            .init_resource::<CbtRenderPages>()
            .init_resource::<CbtRenderSurface>()
            .init_resource::<CbtFrameInput>()
            .init_resource::<CbtFrameOutput>()
            .add_systems(PostUpdate, apply_cbt_frame);
        #[cfg(feature = "render")]
        app.add_plugins(CbtRenderPlugin);
    }
}

fn apply_cbt_frame(
    mut state: ResMut<CbtRenderState>,
    mut input: ResMut<CbtFrameInput>,
    mut output: ResMut<CbtFrameOutput>,
    mut render_topology: ResMut<CbtRenderTopology>,
) {
    let Some((view, candidates, budget)) = input.take() else {
        output.updates.clear();
        output.error = None;
        return;
    };
    if let Some(view) = view {
        state.set_view(view);
    }
    let plan = state.plan_frame(candidates, budget.unwrap_or_else(|| state.frame_budget()));
    let updates = plan.updates().to_vec();
    let result = state.commit(&plan);
    output.frame_index = output.frame_index.saturating_add(1);
    output.updates = updates;
    output.error = result.as_ref().err().map(ToString::to_string);
    let leaf_list = state.leaf_list();
    if result.is_ok() && !plan.is_empty() {
        output.topology_generation = output.topology_generation.saturating_add(1);
        *render_topology = CbtRenderTopology::from_leaf_list(
            output.topology_generation,
            state.topology.max_depth(),
            &leaf_list,
        );
    }
    output.leaf_list = leaf_list;
}

#[cfg(feature = "render")]
mod render;

#[cfg(feature = "render")]
use render::CbtRenderPlugin;

#[cfg(feature = "render")]
pub use render::CbtGpuBuffers;

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::app::App;
    use thessa_rcbt_core::{CandidateAction, Node, WorkClass};

    fn split_root() -> LeafCandidate {
        LeafCandidate {
            node: Node::root(),
            action: CandidateAction::Split,
            class: WorkClass::CoverageRepair,
            projected_error_px: 2.0,
            predicted_error_px: 0.0,
            time_to_needed_s: 1.0,
        }
    }

    #[test]
    fn plugin_commits_submitted_work_after_update() {
        let mut app = App::new();
        app.add_plugins(CbtPlugin {
            max_depth: 8,
            initial_depth: 0,
            frame_budget: FrameBudget { max_operations: 1 },
        });
        app.world_mut()
            .resource_mut::<CbtFrameInput>()
            .submit(None, [split_root()]);
        app.update();
        let state = app.world().resource::<CbtRenderState>();
        assert_eq!(state.topology().leaf_count(), 2);
        assert_eq!(app.world().resource::<CbtFrameOutput>().updates.len(), 1);
    }

    #[test]
    fn plugin_does_not_replay_consumed_work() {
        let mut app = App::new();
        app.add_plugins(CbtPlugin::default());
        app.world_mut()
            .resource_mut::<CbtFrameInput>()
            .submit(None, [split_root()]);
        app.update();
        app.update();
        assert!(app.world().resource::<CbtFrameOutput>().updates.is_empty());
        assert_eq!(app.world().resource::<CbtFrameOutput>().frame_index, 1);
    }

    #[test]
    fn render_records_preserve_deep_node_ids_without_float_conversion() {
        let mut tree = Tree::new(37).unwrap();
        let mut node = Node::root();
        for _ in 0..37 {
            node = node.children().unwrap()[1];
            tree.split(node.parent().unwrap()).unwrap();
        }
        let topology = CbtRenderTopology::from_tree(&tree);
        let record = topology
            .records()
            .iter()
            .find(|record| (u64::from(record[0]) | (u64::from(record[1]) << 32)) == node.id())
            .copied()
            .expect("deep path record");
        let id = u64::from(record[0]) | (u64::from(record[1]) << 32);
        assert_eq!(id, node.id());
        assert_eq!(record[3] as usize, tree.encode_leaf(node).unwrap());
    }

    #[test]
    fn render_pages_have_stable_generation_updates() {
        let page = HeightPage::bake(&[10.0, 10.0, 11.0, 11.0], 2, 0.01).unwrap();
        let mut pages = CbtRenderPages::default();
        pages.set_page(17, page.clone());
        assert_eq!(pages.generation(), 1);
        pages.set_page(17, page);
        assert_eq!(pages.generation(), 1);
        assert_eq!(pages.len(), 1);
        pages.remove_page(17);
        assert_eq!(pages.generation(), 2);
        assert!(pages.is_empty());
    }
}
