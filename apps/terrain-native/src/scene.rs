//! A real canonical-field resident partition, shared by window/headless paths.
//! Shared live terrain streaming policy; not the networked game client.
use glam::DVec3;
use thessa_rcbt_core::tile_anchor::{TileAnchor, TileKey};
use thessa_render_wgpu::{
    Backend, BackendError,
    terrain::{ResidentTerrainPatch, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};
use thessa_terrain_stream::{
    runtime::{StreamUpdate, StreamingPolicy, TerrainStream},
    selection::SelectionView,
};

pub struct Scene {
    pub requested: thessa_graphics::RequestedGraphics,
    pub lighting: thessa_rcbt_core::lighting::SurfaceLighting,
    pub stream: TerrainStream,
    pub publications: u64,
    pub regenerated: u64,
    pub material_upload_bytes: u64,
    pub capacity_rejections: u64,
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
        let stream = TerrainStream::new(field, StreamingPolicy::default())?;
        eprintln!(
            "canonical frozen Thessa: {} resident tiles; highlands={:?}",
            stream.visible().len(),
            center
        );
        Ok(Self {
            requested,
            lighting,
            stream,
            publications: 0,
            regenerated: 0,
            material_upload_bytes: 0,
            capacity_rejections: 0,
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
            .stream
            .visible()
            .iter()
            .map(|&key| TerrainPatch {
                anchor: self.anchor(key),
                page: &self.stream.height(key).unwrap().page,
            })
            .collect();
        let mut renderer = TerrainRenderer::with_capacity(
            backend,
            &patches,
            format,
            TerrainCapacity {
                patches: 1024,
                height_grid_size: 33,
                material_layers: backend.device.limits().max_texture_array_layers.min(256),
            },
        )?;
        let stats = renderer.update_materials(
            backend,
            self.stream.material_demand(),
            self.stream.materials(),
        )?;
        renderer.update_lighting(backend, &self.lighting)?;
        eprintln!(
            "material publication: {} pages, {} bytes",
            stats.pages, stats.bytes
        );
        Ok(renderer)
    }

    fn anchor(&self, key: thessa_worldgen_rocky::lod::TileKey) -> TileAnchor {
        TileAnchor::new(
            TileKey::new(key.face, key.level, key.x, key.y).unwrap(),
            self.stream_radius(),
        )
        .unwrap()
    }

    fn stream_radius(&self) -> f64 {
        self.stream.radius_m()
    }

    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        backend: &Backend,
        renderer: &mut TerrainRenderer,
        width: u32,
        height: u32,
        exposure: f32,
        now_s: f64,
    ) -> Result<StreamUpdate, Box<dyn std::error::Error>> {
        let camera = self.view(width, height, exposure);
        let forward = camera
            .world_from_view
            .transform_vector3(DVec3::NEG_Z)
            .normalize();
        let half_angle = 0.45f64.max((0.45f64.tan() * width as f64 / height as f64).atan()) + 0.6;
        let update = self.stream.tick(
            SelectionView {
                eye: camera.world_position_m,
                stream_origin: self.center,
                forward,
                radius: self.stream_radius(),
                frustum: Some(thessa_worldgen_rocky::lod::SelectionFrustum {
                    forward: forward.to_array(),
                    cos_limit: half_angle.cos().clamp(-1.0, 1.0),
                }),
            },
            2.0 * 0.45f64.tan() / height as f64,
            now_s,
        )?;
        if update.geometry_changed {
            let patches: Vec<_> = self
                .stream
                .visible()
                .iter()
                .map(|&key| {
                    let entry = self.stream.height(key).unwrap();
                    ResidentTerrainPatch {
                        anchor: self.anchor(key),
                        page: &entry.page,
                        generation: entry.generation,
                    }
                })
                .collect();
            let stats = renderer.publish_partition(backend, &patches)?;
            self.publications += 1;
            self.regenerated += stats.regenerated as u64;
        }
        if update.materials_changed || update.geometry_changed {
            let stats = renderer.update_materials(
                backend,
                self.stream.material_demand(),
                self.stream.materials(),
            )?;
            self.material_upload_bytes += stats.bytes as u64;
        }
        self.capacity_rejections += u64::from(update.capacity_rejected);
        Ok(update)
    }

    pub fn view(&self, width: u32, height: u32, exposure: f32) -> ViewParams {
        orbit_view(
            self.center,
            self.up,
            self.east,
            self.distance,
            self.angle,
            width,
            height,
            exposure,
        )
    }
}

#[allow(clippy::too_many_arguments)]
fn orbit_view(
    center: DVec3,
    up: DVec3,
    east: DVec3,
    distance: f64,
    angle: f64,
    width: u32,
    height: u32,
    exposure: f32,
) -> ViewParams {
    let north = east.cross(up);
    let tangent = north * angle.cos() + east * angle.sin();
    let eye = center + (up - tangent) * distance;
    ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, north).inverse(),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orbit_views_keep_checked_camera_relative_abi() {
        let center = DVec3::Z * 3_200_000.0;
        for distance in [1_000.0, 30_000.0, 200_000.0] {
            for step in 0..32 {
                let view = orbit_view(
                    center,
                    DVec3::Z,
                    DVec3::X,
                    distance,
                    step as f64 * std::f64::consts::TAU / 32.0,
                    1280,
                    720,
                    1.0,
                );
                assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
                assert!(
                    (view.world_from_view.w_axis.truncate() - view.world_position_m).length()
                        < 1e-6
                );
            }
        }
    }
}
