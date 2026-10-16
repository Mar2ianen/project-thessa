//! A real canonical-field resident partition, shared by window/headless paths.
//! Shared live terrain streaming policy; not the networked game client.
use crate::celestial::{CelestialPresentation, Sample};
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
    celestial: CelestialPresentation,
    pub sample: Sample,
    lighting_dirty: bool,
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
        let body = ephemeris
            .body_id(&recipe.planet.id)
            .ok_or("missing terrain body")?;
        let celestial = CelestialPresentation::new(&system, ephemeris, body)?;
        let sample = celestial.sample(thessa_sim_core::SimTime::EPOCH)?;
        let up = DVec3::from(sites[1]); // same HIGHLANDS bookmark as the shipped client
        let center = up * (field.params.radius_m + field.height_m(up.to_array(), 32.0).max(0.0));
        let east = DVec3::Y.cross(up).normalize();
        let stream = TerrainStream::new(field, StreamingPolicy::default())?;
        eprintln!(
            "canonical Thessa: {} resident tiles; highlands={:?}",
            stream.visible().len(),
            center
        );
        Ok(Self {
            requested,
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
            celestial,
            sample,
            lighting_dirty: false,
        })
    }

    /// Explicit presentation sampling, separate from wall-time streaming
    /// debounce. An unchanged SimTime performs no ephemeris or lighting work.
    /// Stage the full sample before mutation so failure preserves the old frame.
    pub fn set_presentation_time(&mut self, time: thessa_sim_core::SimTime) -> Result<(), String> {
        if time == self.sample.time {
            return Ok(());
        }
        let sample = self.celestial.sample(time)?;
        self.lighting_dirty |= self.sample.lighting != sample.lighting;
        self.sample = sample;
        Ok(())
    }

    fn publish_presentation(
        &mut self,
        backend: &Backend,
        renderer: &mut TerrainRenderer,
    ) -> Result<(), BackendError> {
        if self.lighting_dirty {
            renderer.update_lighting(backend, &self.sample.lighting)?;
            self.lighting_dirty = false;
        }
        Ok(())
    }

    pub fn renderer(
        &mut self,
        backend: &Backend,
        format: wgpu::TextureFormat,
    ) -> Result<TerrainRenderer, BackendError> {
        let patches: Vec<_> = self
            .stream
            .visible()
            .iter()
            .map(|&key| TerrainPatch {
                anchor: self.anchor(key),
                page: self.stream.height(key).unwrap().page,
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
        renderer.update_lighting(backend, &self.sample.lighting)?;
        self.lighting_dirty = false;
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
        self.publish_presentation(backend, renderer)?;
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
        if update.height_failed > 0 {
            eprintln!(
                "[terrain] {} height worker(s) failed; retrying demand",
                update.height_failed
            );
        }
        if update.material_failed > 0 {
            eprintln!(
                "[terrain] {} material worker(s) disconnected; retrying demand",
                update.material_failed
            );
        }
        if update.geometry_changed {
            let patches: Vec<_> = self
                .stream
                .visible()
                .iter()
                .map(|&key| {
                    let entry = self.stream.height(key).unwrap();
                    ResidentTerrainPatch {
                        anchor: self.anchor(key),
                        page: entry.page,
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
        let frame = self.sample.frame;
        let render_view = orbit_view(
            frame.body_to_render_point(self.center),
            frame.body_to_render_direction(self.up),
            frame.body_to_render_direction(self.east),
            self.distance,
            self.angle,
            width,
            height,
            exposure,
        );
        frame.view_in_body(&render_view)
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
    #[ignore = "requires a wgpu adapter"]
    fn sampled_motion_updates_gpu_lighting_without_rebuilding_height_geometry() {
        let backend = Backend::bring_up().unwrap();
        let mut scene = Scene::load(thessa_graphics::RequestedGraphics::default()).unwrap();
        let target = thessa_render_wgpu::terrain::OffscreenTarget::new(&backend, 257, 257).unwrap();
        let mut renderer = scene
            .renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap();
        let geometry = renderer.read_vertices(&backend).unwrap();
        let exposure = crate::manual_exposure(13.0);
        let render = |scene: &Scene, renderer: &mut TerrainRenderer| {
            renderer
                .update_view(&backend, &scene.view(257, 257, exposure))
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            renderer.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            target.read_rgba8(&backend).unwrap()
        };
        let before = render(&scene, &mut renderer);
        scene
            .set_presentation_time(thessa_sim_core::SimTime(72_000.0))
            .unwrap();
        // Isolate the pose/light publication stage: initial terrain selection
        // itself can legitimately remove out-of-view bootstrap roots.
        scene.publish_presentation(&backend, &mut renderer).unwrap();
        assert!(!scene.lighting_dirty);
        assert_eq!(renderer.read_vertices(&backend).unwrap(), geometry);
        let after = render(&scene, &mut renderer);
        assert_ne!(
            before, after,
            "new sample lighting never reached the terrain GPU pass"
        );
    }

    #[test]
    fn moving_celestial_samples_keep_body_attached_camera_and_failed_sample_preserves_state() {
        let mut scene = Scene::load(thessa_graphics::RequestedGraphics::default()).unwrap();
        let epoch = scene.view(1280, 720, 1.0);
        let initial_lighting = scene.sample.lighting;
        for time in [1.0, 72_000.0, 144_000.0, -72_000.0] {
            scene
                .set_presentation_time(thessa_sim_core::SimTime(time))
                .unwrap();
            let view = scene.view(1280, 720, 1.0);
            assert!((view.world_position_m - epoch.world_position_m).length() < 1e-8);
            assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
            let forward = view.world_from_view.transform_vector3(DVec3::NEG_Z);
            let original_forward = epoch.world_from_view.transform_vector3(DVec3::NEG_Z);
            assert!((forward - original_forward).length() < 1e-12);
        }
        assert_ne!(scene.sample.lighting, initial_lighting);
        let before = scene.sample;
        let lighting = scene.sample.lighting;
        assert!(
            scene
                .set_presentation_time(thessa_sim_core::SimTime(f64::NAN))
                .is_err()
        );
        assert_eq!(scene.sample.time, before.time);
        assert_eq!(scene.sample.frame, before.frame);
        assert_eq!(scene.sample.lighting, lighting);
        scene.lighting_dirty = false;
        scene.set_presentation_time(before.time).unwrap();
        assert!(
            !scene.lighting_dirty,
            "same-time samples must be free of dirty work"
        );
    }

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
