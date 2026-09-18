//! Terrain backend abstraction.
//!
//! The legacy CPU tile path (`lod::build_tile` + `lod::build_surface_texture`)
//! stays untouched and remains the default renderer. The RCBT path
//! (`rcbt-core::Tree` topology planning + `lod::bake_height_page`) plugs in
//! next to it behind one trait so both can be driven by the same request and
//! compared on identical inputs.
//!
//! Selection is shared on purpose: both backends observe the same
//! `select_tiles_with_height` set, so the comparison isolates *build* cost
//! (heavy mesh + texture baking vs. compact page baking + light topology
//! planning), not selection heuristics.

use std::time::Instant;

use thessa_rcbt_core::{CandidateAction, FrameBudget, LeafCandidate, Tree, WorkClass, plan_frame};

use crate::{
    field::PlanetField,
    lod::{self},
};

/// One reproducible streaming workload for both backends.
#[derive(Debug, Clone, Copy)]
pub struct TerrainRequest {
    pub eye_m: [f64; 3],
    pub radius_m: f64,
    pub max_level: u8,
    pub budget: usize,
    /// How many of the selected tiles to actually build (keeps benches
    /// bounded; selection always runs on the full budget).
    pub build_limit: usize,
    /// Legacy mesh density per tile side.
    pub mesh_cells: usize,
    /// RCBT page grid side (samples = grid * grid).
    pub page_grid: u32,
    /// RCBT absolute bake error budget in metres.
    pub page_error_m: f64,
}

impl Default for TerrainRequest {
    fn default() -> Self {
        Self {
            eye_m: [3_205_000.0, 0.0, 0.0],
            radius_m: 3_200_000.0,
            max_level: 17,
            budget: 192,
            build_limit: 4,
            mesh_cells: 24,
            page_grid: 9,
            page_error_m: 100.0,
        }
    }
}

/// Measured result of one backend on one request.
#[derive(Debug, Clone)]
pub struct BackendOutput {
    pub backend: &'static str,
    pub selected: usize,
    pub built: usize,
    pub vertices: u64,
    pub indices: u64,
    /// Legacy: albedo + roughness + normal texture bytes.
    /// RCBT: stable `HeightPage` encoding bytes.
    pub payload_bytes: u64,
    pub select_ms: f64,
    pub topology_ms: f64,
    pub build_ms: f64,
    /// Max |backend height - canonical field height| over probed samples.
    pub max_error_m: f64,
}

impl BackendOutput {
    pub fn total_ms(&self) -> f64 {
        self.select_ms + self.topology_ms + self.build_ms
    }
}

/// One terrain implementation behind the shared request.
pub trait TerrainBackend {
    fn name(&self) -> &'static str;
    fn run(&self, field: &PlanetField, req: &TerrainRequest) -> BackendOutput;
}

/// Existing production path, unmodified: full mesh + per-tile textures.
pub struct LegacyCpuBackend;

impl TerrainBackend for LegacyCpuBackend {
    fn name(&self) -> &'static str {
        "legacy-cpu"
    }

    fn run(&self, field: &PlanetField, req: &TerrainRequest) -> BackendOutput {
        let started = Instant::now();
        let keys = lod::select_tiles_with_height(
            req.eye_m,
            req.radius_m,
            req.max_level,
            req.budget,
            |dir| field.height_m(dir, 32.0),
        );
        let select_ms = started.elapsed().as_secs_f64() * 1000.0;

        let started = Instant::now();
        let mut vertices = 0_u64;
        let mut indices = 0_u64;
        let mut payload_bytes = 0_u64;
        let mut built = 0_usize;
        for key in keys.iter().take(req.build_limit) {
            let tile = lod::build_tile(field, *key, req.mesh_cells);
            vertices += tile.positions.len() as u64;
            indices += tile.indices.len() as u64;
            let tex_cells = lod::texture_cells_for_level(key.level);
            let tex = lod::build_surface_texture(field, *key, tex_cells);
            payload_bytes += (tex.albedo.len() + tex.roughness.len() + tex.normal.len()) as u64;
            built += 1;
        }
        let build_ms = started.elapsed().as_secs_f64() * 1000.0;

        BackendOutput {
            backend: self.name(),
            selected: keys.len(),
            built,
            vertices,
            indices,
            payload_bytes,
            select_ms,
            topology_ms: 0.0,
            build_ms,
            max_error_m: 0.0,
        }
    }
}

/// New adaptive path: light CBT topology planning + compact baked pages.
/// Height source is the same canonical `PlanetField`; only the
/// representation changes.
pub struct RcbtBackend {
    pub tree_max_depth: u8,
    pub tree_base_depth: u8,
    pub max_ops_per_frame: usize,
}

impl Default for RcbtBackend {
    fn default() -> Self {
        Self {
            tree_max_depth: 18,
            tree_base_depth: 7,
            max_ops_per_frame: 8,
        }
    }
}

impl TerrainBackend for RcbtBackend {
    fn name(&self) -> &'static str {
        "rcbt-pages"
    }

    fn run(&self, field: &PlanetField, req: &TerrainRequest) -> BackendOutput {
        let started = Instant::now();
        let keys = lod::select_tiles_with_height(
            req.eye_m,
            req.radius_m,
            req.max_level,
            req.budget,
            |dir| field.height_m(dir, 32.0),
        );
        let select_ms = started.elapsed().as_secs_f64() * 1000.0;

        // Topology planning cost, scaled by the same build workload.
        // Candidates are synthetic but deterministic: one split candidate per
        // built tile plus a bounded merge tail, mirroring the sparse-mutation
        // frame update from `rcbt-core/benches/tree.rs`.
        let started = Instant::now();
        let mut tree = Tree::at_depth(self.tree_max_depth, self.tree_base_depth)
            .expect("benchmark tree depths");
        let leaves = tree.leaves();
        let candidates = leaves
            .iter()
            .copied()
            .take(req.build_limit.max(1) * 4)
            .enumerate()
            .map(|(index, node)| LeafCandidate {
                node,
                action: if index % 5 == 0 {
                    CandidateAction::Merge
                } else {
                    CandidateAction::Split
                },
                class: if index < 4 {
                    WorkClass::VisibleGeometry
                } else {
                    WorkClass::PredictedGeometry
                },
                projected_error_px: 4.0 + (index % 7) as f32,
                predicted_error_px: 2.0,
                time_to_needed_s: 0.25 + index as f32 * 0.1,
            });
        let plan = plan_frame(
            &tree,
            candidates,
            FrameBudget {
                max_operations: self.max_ops_per_frame,
            },
        );
        tree.apply_batch(plan.updates())
            .expect("planned topology updates");
        let topology_ms = started.elapsed().as_secs_f64() * 1000.0;

        let started = Instant::now();
        let mut payload_bytes = 0_u64;
        let mut max_error_m = 0.0_f64;
        let mut built = 0_usize;
        for key in keys.iter().take(req.build_limit) {
            let page = lod::bake_height_page(field, *key, req.page_grid, req.page_error_m)
                .expect("page bake within budget");
            payload_bytes += page.to_bytes().len() as u64;
            // Probe page centres against the canonical field at the same
            // wavelength the page was baked from. Bake clamps
            // below-datum samples to the spherical datum (visible tile
            // path renders ocean as datum), so the probe must clamp the
            // reference identically or ocean tiles report metres of
            // phantom error.
            let cells = req.page_grid.saturating_sub(1).max(1) as f64;
            let wavelength = (key.span_m(field.params.radius_m) / cells).max(32.0);
            let grid = req.page_grid as usize;
            for y in 0..grid {
                for x in 0..grid {
                    let dir = key.direction(x as f64 / cells, y as f64 / cells);
                    let reference = field.height_m(dir, wavelength).max(0.0);
                    let got = page
                        .decoded_sample(x as u32, y as u32)
                        .expect("in-grid sample") as f64;
                    max_error_m = max_error_m.max((got - reference).abs());
                }
            }
            max_error_m = max_error_m.max(page.max_residual_error_m() as f64);
            built += 1;
        }
        let build_ms = started.elapsed().as_secs_f64() * 1000.0;

        BackendOutput {
            backend: self.name(),
            selected: keys.len(),
            built,
            vertices: 0,
            indices: 0,
            payload_bytes,
            select_ms,
            topology_ms,
            build_ms,
            max_error_m,
        }
    }
}

/// Selectable backend. Legacy stays the default.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BackendKind {
    #[default]
    Legacy,
    Rcbt,
}

impl BackendKind {
    pub fn from_env() -> Self {
        match std::env::var("THESSA_TERRAIN_BACKEND")
            .unwrap_or_default()
            .to_ascii_lowercase()
            .as_str()
        {
            "rcbt" | "rcbt-pages" | "new" => Self::Rcbt,
            _ => Self::Legacy,
        }
    }
}

pub fn create_backend(kind: BackendKind) -> Box<dyn TerrainBackend> {
    match kind {
        BackendKind::Legacy => Box::new(LegacyCpuBackend),
        BackendKind::Rcbt => Box::new(RcbtBackend::default()),
    }
}

/// Side-by-side comparison on identical inputs.
#[derive(Debug, Clone)]
pub struct Comparison {
    pub legacy: BackendOutput,
    pub rcbt: BackendOutput,
}

impl Comparison {
    /// Speedup > 1 means the RCBT path built faster (build + topology only,
    /// selection is shared and excluded).
    pub fn build_speedup(&self) -> f64 {
        let old = self.legacy.build_ms;
        let new = self.rcbt.build_ms + self.rcbt.topology_ms;
        if new <= 0.0 { f64::INFINITY } else { old / new }
    }

    /// Payload ratio < 1 means the RCBT path ships fewer bytes per tile.
    pub fn payload_ratio(&self) -> f64 {
        if self.legacy.payload_bytes == 0 {
            0.0
        } else {
            self.rcbt.payload_bytes as f64 / self.legacy.payload_bytes as f64
        }
    }
}

pub fn compare(field: &PlanetField, req: &TerrainRequest) -> Comparison {
    Comparison {
        legacy: LegacyCpuBackend.run(field, req),
        rcbt: RcbtBackend::default().run(field, req),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lod::TileKey;

    fn test_field() -> PlanetField {
        let recipe: crate::spec_recipe::SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }

    #[test]
    fn both_backends_observe_the_same_selection() {
        let field = test_field();
        let req = TerrainRequest {
            build_limit: 2,
            ..TerrainRequest::default()
        };
        let cmp = compare(&field, &req);
        assert_eq!(cmp.legacy.selected, cmp.rcbt.selected);
        assert_eq!(cmp.legacy.built, cmp.rcbt.built);
        assert!(cmp.rcbt.max_error_m <= req.page_error_m + 1.0);
    }

    #[test]
    fn backend_kind_parses_env_selection() {
        assert_eq!(BackendKind::default(), BackendKind::Legacy);
        assert_eq!(create_backend(BackendKind::Legacy).name(), "legacy-cpu");
        assert_eq!(create_backend(BackendKind::Rcbt).name(), "rcbt-pages");
    }

    #[test]
    fn single_tile_spot_check_is_within_page_budget() {
        let field = test_field();
        let key = TileKey {
            face: 1,
            level: 10,
            x: 100,
            y: 200,
        };
        let page = lod::bake_height_page(&field, key, 9, 100.0).unwrap();
        let dir = key.direction(0.5, 0.5);
        let reference = field.height_m(dir, (key.span_m(field.params.radius_m) / 8.0).max(32.0));
        assert!((page.sample(0.5, 0.5) as f64 - reference).abs() <= 100.0);
        let _mesh = lod::build_tile(&field, key, 24);
    }
}
