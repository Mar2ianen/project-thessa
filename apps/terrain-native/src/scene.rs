//! A real canonical-field resident partition, shared by window/headless paths.
//! This is a bounded static streaming snapshot, not the networked game client.
use glam::DVec3;
use rayon::prelude::*;
use thessa_rcbt_core::{
    HeightPage,
    material_page::{CbtMaterialPage, MaterialPages},
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend, BackendError,
    terrain::{TerrainPatch, TerrainRenderer},
    view::ViewParams,
};
use thessa_worldgen_rocky::lod;

pub struct Scene {
    pub requested: thessa_graphics::RequestedGraphics,
    pub lighting: thessa_rcbt_core::lighting::SurfaceLighting,
    pub anchors: Vec<TileAnchor>,
    pub heights: Vec<HeightPage>,
    pub materials: MaterialPages,
    pub demand: Vec<u64>,
    pub center: DVec3,
    pub up: DVec3,
    pub east: DVec3,
    pub distance: f64,
    pub angle: f64,
}

impl Scene {
    pub fn load(requested: thessa_graphics::RequestedGraphics) -> Result<Self, String> {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).map_err(|e| e.to_string())?;
        let ephemeris = system.bake().map_err(|e| e.to_string())?;
        let (field, sites) = thessa_flight_authority::canonical_launch_setup(&ephemeris)?;
        let recipe: thessa_worldgen_rocky::spec_recipe::SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))
                .map_err(|e| e.to_string())?;
        let planet = ephemeris
            .body_state(
                ephemeris
                    .body_id(&recipe.planet.id)
                    .ok_or("missing terrain body")?,
                thessa_sim_core::SimTime::EPOCH,
            )
            .map_err(|e| e.to_string())?;
        let mut stars = Vec::new();
        for star in &system.star {
            let state = ephemeris
                .body_state(
                    ephemeris.body_id(&star.id).ok_or("missing star")?,
                    thessa_sim_core::SimTime::EPOCH,
                )
                .map_err(|e| e.to_string())?;
            let delta = state.position_inertial - planet.position_inertial;
            let direction = delta.normalize();
            // Same inertial -> terrain axes as the launch bookmark scan; at
            // EPOCH the body's render spin is zero. No fabricated local sun.
            let body_direction = [
                direction.x as f32,
                direction.z as f32,
                -direction.y as f32,
                0.0,
            ];
            let lux = thessa_atmosphere::irradiance_at_distance(
                star.luminosity_solar * 3.828e26,
                delta.length(),
            ) * 93.0;
            let tint = thessa_atmosphere::blackbody_rgb(star.temperature_k);
            stars.push((
                lux,
                body_direction,
                [
                    (tint[0] * lux) as f32,
                    (tint[1] * lux) as f32,
                    (tint[2] * lux) as f32,
                    0.0,
                ],
            ));
        }
        stars.sort_by(|a, b| b.0.total_cmp(&a.0));
        if stars.len() > 3 {
            return Err(
                "terrain shader currently supports at most three direct stellar sources".into(),
            );
        }
        let mut lighting = thessa_rcbt_core::lighting::SurfaceLighting::default();
        for (index, (_, direction, color)) in stars.into_iter().enumerate() {
            lighting.directions[index] = direction;
            lighting.colors_lux[index] = color;
        }
        let up = DVec3::from(sites[1]); // same HIGHLANDS bookmark as the shipped client
        let center = up * (field.params.radius_m + field.height_m(up.to_array(), 32.0).max(0.0));
        let east = DVec3::Y.cross(up).normalize();
        let eye = center + up * 30_000.0 - east.cross(up) * 30_000.0;
        let tiles =
            lod::select_tiles_with_height(eye.to_array(), field.params.radius_m, 12, 64, |d| {
                field.height_m(d, 500.0).max(0.0)
            });
        let built: Vec<_> = tiles
            .par_iter()
            .map(|&key| {
                let height =
                    lod::bake_height_page(&field, key, 33, 0.5).map_err(|e| e.to_string())?;
                let material =
                    CbtMaterialPage::from_rgba8(lod::build_gpu_material_page(&field, key).rgba)
                        .ok_or("invalid material shape")?;
                let anchor = TileAnchor::new(
                    TileKey::new(key.face, key.level, key.x, key.y).ok_or("invalid tile")?,
                    field.params.radius_m,
                )
                .ok_or("invalid anchor")?;
                let id = lod::cbt_node_for_tile(key).ok_or("invalid CBT node")?.id();
                Ok((anchor, height, id, material))
            })
            .collect::<Result<_, String>>()?;
        let mut anchors = Vec::new();
        let mut heights = Vec::new();
        let mut materials = MaterialPages::new();
        let mut demand = Vec::new();
        for (anchor, height, id, material) in built {
            anchors.push(anchor);
            heights.push(height);
            demand.push(id);
            materials.insert(id, (1, material));
        }
        // The production shader feathers missing neighbours into common coarse
        // pages. Supply the resident ancestors too, rather than exposing the
        // diagnostic globe color at every mixed-LOD tile boundary.
        let mut ancestors = std::collections::BTreeSet::new();
        for &id in &demand {
            let mut parent = id >> 2;
            while parent >= 8 {
                if !materials.contains_key(&parent) {
                    ancestors.insert(parent);
                }
                parent >>= 2;
            }
        }
        let coarse: Vec<_> = ancestors
            .into_par_iter()
            .map(|id| {
                let node = thessa_rcbt_core::Node::from_heap_id(id).map_err(|e| e.to_string())?;
                let key = lod::tile_for_cbt_node(node).ok_or("invalid material ancestor")?;
                let page =
                    CbtMaterialPage::from_rgba8(lod::build_gpu_material_page(&field, key).rgba)
                        .ok_or("invalid material shape")?;
                Ok((id, (1, page)))
            })
            .collect::<Result<_, String>>()?;
        materials.extend(coarse);
        eprintln!(
            "canonical frozen Thessa: {} resident tiles; highlands={:?}",
            anchors.len(),
            center
        );
        Ok(Self {
            requested,
            lighting,
            anchors,
            heights,
            materials,
            demand,
            center,
            up,
            east,
            distance: 30_000.0,
            angle: 0.0,
        })
    }

    pub fn renderer(
        &self,
        backend: &Backend,
        format: wgpu::TextureFormat,
    ) -> Result<TerrainRenderer, BackendError> {
        let patches: Vec<_> = self
            .anchors
            .iter()
            .zip(&self.heights)
            .map(|(&anchor, page)| TerrainPatch { anchor, page })
            .collect();
        let mut renderer = TerrainRenderer::with_material_capacity(
            backend,
            &patches,
            format,
            self.materials.len() as u32,
        )?;
        let stats = renderer.update_materials(backend, &self.demand, &self.materials)?;
        renderer.update_lighting(backend, &self.lighting)?;
        eprintln!(
            "material publication: {} pages, {} bytes",
            stats.pages, stats.bytes
        );
        Ok(renderer)
    }

    pub fn view(&self, width: u32, height: u32, exposure: f32) -> ViewParams {
        let north = self.east.cross(self.up);
        let tangent = north * self.angle.cos() + self.east * self.angle.sin();
        let eye = self.center + (self.up - tangent) * self.distance;
        ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, self.center, north)
                .inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                0.9,
                width as f64 / height as f64,
                1.0,
                1e8,
            ),
            world_position_m: eye,
            exposure,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orbit_views_keep_checked_camera_relative_abi() {
        let mut scene = Scene {
            requested: Default::default(),
            lighting: Default::default(),
            anchors: vec![],
            heights: vec![],
            materials: MaterialPages::new(),
            demand: vec![],
            center: DVec3::Z * 3_200_000.0,
            up: DVec3::Z,
            east: DVec3::X,
            distance: 30_000.0,
            angle: 0.0,
        };
        for distance in [1_000.0, 30_000.0, 200_000.0] {
            for step in 0..32 {
                scene.distance = distance;
                scene.angle = step as f64 * std::f64::consts::TAU / 32.0;
                let view = scene.view(1280, 720, 1.0);
                assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
                assert!(
                    (view.world_from_view.w_axis.truncate() - view.world_position_m).length()
                        < 1e-6
                );
            }
        }
    }
}
