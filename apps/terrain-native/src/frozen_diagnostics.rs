//! Paired diagnostic draws of one already-published native frame. No stream,
//! simulation, topology, material, shadow or sky publication between views.
use std::{error::Error, path::Path};
use thessa_graphics::{ResolvedGraphicsSettings, TerrainDebugView as V};
use thessa_render_wgpu::{
    Backend,
    display::DisplayTransform,
    msaa::SceneMsaa,
    terrain::{OffscreenTarget, TerrainRenderer},
};

pub struct Capture<'a> {
    pub backend: &'a Backend,
    pub scene: &'a crate::scene::Scene,
    pub renderer: &'a TerrainRenderer,
    pub meshes: Option<&'a crate::scene::SceneMeshes>,
    pub display: &'a DisplayTransform,
    pub sky: &'a crate::sky_layers::SkyLayers,
    pub msaa: Option<&'a SceneMsaa>,
    pub ambient_visibility:
        Option<&'a mut thessa_render_wgpu::ambient_visibility::AmbientVisibility>,
    pub target: &'a OffscreenTarget,
    pub settings: &'a ResolvedGraphicsSettings,
    pub exposure: f32,
    pub trace: Option<&'a mut crate::gpu_trace::GpuTrace>,
}

fn vertices(renderer: &TerrainRenderer, backend: &Backend) -> Result<Vec<u8>, Box<dyn Error>> {
    Ok(renderer
        .read_vertices(backend)?
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .collect())
}

impl Capture<'_> {
    pub fn run(&mut self, directory: &Path) -> Result<(), Box<dyn Error>> {
        if self.renderer.conforming_surface().is_none()
            || self.renderer.terrain_debug_storage_bytes() != 16
        {
            return Err("frozen diagnostics require an explicitly enabled diagnostic renderer with conforming CBT".into());
        }
        if self.settings.ray_tracing.is_active()
            || self.settings.temporal_taa
            || self.settings.temporal_motion_vectors
            || self.settings.temporal_jitter
        {
            return Err(
                "frozen diagnostic sequence currently requires RT Off and temporal effects Off"
                    .into(),
            );
        }
        let hdr = self
            .display
            .scene_texture()
            .ok_or("frozen diagnostic sequence requires resolved HDR")?;
        let size = hdr.size();
        std::fs::create_dir(directory)?;
        let epoch = self.scene.temporal_epoch(self.renderer, self.meshes);
        let geometry = vertices(self.renderer, self.backend)?;
        let topology = self.renderer.read_topology_records(self.backend)?;
        let topology_state = self.renderer.read_topology_state(self.backend)?;
        let draw = self.renderer.read_draw(self.backend)?;
        std::fs::write(directory.join("terrain-vertices.bin"), &geometry)?;
        if let Some(records) = &topology {
            std::fs::write(
                directory.join("topology.bin"),
                records
                    .iter()
                    .flat_map(|r| r.address.into_iter().chain(r.neighbors))
                    .flat_map(u32::to_le_bytes)
                    .collect::<Vec<_>>(),
            )?;
        }
        std::fs::write(
            directory.join("requested.toml"),
            toml::to_string(&self.scene.requested)?,
        )?;
        std::fs::write(
            directory.join("resolved.json"),
            serde_json::to_vec_pretty(self.settings)?,
        )?;
        std::fs::write(
            directory.join("worldgen_recipe.toml"),
            include_str!("../../../data/worldgen/worldgen_recipe.toml"),
        )?;
        std::fs::write(
            directory.join("system.toml"),
            include_str!("../../../data/system.toml"),
        )?;
        let view = self.scene.view(size.width, size.height, self.exposure);
        let lighting = self.scene.lighting();
        let mut manifest = serde_json::json!({
            "version":1,"complete":false,"performance_equivalent":false,
            "scope":"in-process frozen published frame; independent process reload is not implemented",
            "extent":[size.width,size.height],"sample_count":self.settings.raster_msaa_samples,
            "sim_time_s":self.scene.sample.time.0,"exposure":self.exposure,
            "camera_m":view.world_position_m.to_array(),"world_from_view":view.world_from_view.to_cols_array(),"clip_from_view":view.clip_from_view.to_cols_array(),
            "lighting":{"ambient":lighting.ambient_lux,"directions":lighting.directions,"colors":lighting.colors_lux},
            "epochs":{"terrain":epoch.terrain,"plants":epoch.plants,"presentation":epoch.presentation,"view":epoch.view},
            "height_versions":self.scene.stream.visible().iter().map(|key| serde_json::json!({"face":key.face,"level":key.level,"x":key.x,"y":key.y,"generation":self.scene.stream.height(*key).unwrap().generation})).collect::<Vec<_>>(),
            "material_source_versions":self.scene.stream.materials().iter().map(|(&id,(generation,_))|[id,*generation]).collect::<Vec<_>>(),
            "material_resident_levels":self.scene.material_resident_levels,
            "pending_jobs":self.scene.stream.pending_jobs(),"draw":draw,"topology_state":topology_state,
            "mesh_stats":format!("{:?}", self.meshes.map(|m|m.draw_stats())),"views":[],
        });
        let manifest_path = directory.join("manifest.json");
        std::fs::write(&manifest_path, serde_json::to_vec_pretty(&manifest)?)?;
        let mut reference_hdr = [None, None];
        let mut reference_png = [None, None];
        let mut reference_ambient = None;
        let ambient_gpu = self
            .ambient_visibility
            .as_ref()
            .and_then(|_| thessa_render_wgpu::timing::GpuIntervals::new(self.backend, 2));
        let mut reference_depth = None;
        let modes = [
            V::None,
            V::WaterWeight,
            V::Height,
            V::Distance,
            V::Normal,
            V::Albedo,
            V::Roughness,
            V::Geometry,
            V::HeightPageLevel,
            V::Footprint,
            V::MaterialLod,
            V::ResolvedMaterialLod,
            V::ReliefWeight,
            V::Direct,
            V::Ambient,
            V::Specular,
            V::LandOnly,
            V::None,
        ];
        let mut sequence: Vec<_> = modes.into_iter().map(|mode| (mode, true)).collect();
        if self.ambient_visibility.is_some() {
            sequence.splice(0..0, [(V::None, false), (V::None, true), (V::None, false)]);
        }
        let view_count = sequence.len();
        for (index, (mode, ambient_enabled)) in sequence.into_iter().enumerate() {
            let mut request = self.scene.requested.clone();
            request.debug.terrain_view = mode;
            let settings = self.backend.resolve_graphics(&request);
            self.renderer
                .set_terrain_debug_view(self.backend, &settings)?;
            let label = format!(
                "{index:02}-{}",
                serde_json::to_value(mode)?
                    .as_str()
                    .ok_or("invalid diagnostic name")?
            );
            let mut encoder = self
                .backend
                .device
                .create_command_encoder(&Default::default());
            self.renderer.encode_opaque_scene_with_ambient_ops(
                &mut encoder,
                self.msaa
                    .map_or(self.display.scene_color(&self.target.color), |m| &m.color),
                self.msaa.map_or(&self.target.depth, SceneMsaa::depth),
                self.meshes.and_then(crate::scene::SceneMeshes::opaque),
                true,
                self.msaa
                    .map(|_| self.display.scene_color(&self.target.color)),
                self.ambient_visibility.as_deref(),
            );
            if let Some(msaa) = self.msaa {
                msaa.resolve_depth(&mut encoder, &self.target.depth);
            }
            if let Some(ambient) = &mut self.ambient_visibility {
                if let Some(gpu) = &ambient_gpu {
                    gpu.mark(&mut encoder, 0);
                }
                ambient.encode(
                    self.backend,
                    &mut encoder,
                    self.msaa
                        .map_or(self.display.scene_color(&self.target.color), |s| &s.color),
                    self.msaa
                        .map(|_| self.display.scene_color(&self.target.color)),
                    &self.target.depth,
                    self.msaa.map(SceneMsaa::depth),
                    &view,
                    thessa_render_core::temporal::JitterOffset::ZERO,
                    ambient_enabled,
                )?;
                if let Some(gpu) = &ambient_gpu {
                    gpu.mark(&mut encoder, 1);
                }
                self.renderer.encode_background_transparent(
                    &mut encoder,
                    self.msaa
                        .map_or(self.display.scene_color(&self.target.color), |s| &s.color),
                    self.msaa.map_or(&self.target.depth, SceneMsaa::depth),
                    self.msaa
                        .map(|_| self.display.scene_color(&self.target.color)),
                    self.meshes.and_then(crate::scene::SceneMeshes::opaque),
                );
            }
            if let Some(trace) = &mut self.trace {
                trace.checkpoint(
                    self.backend,
                    &mut encoder,
                    index as u64,
                    &format!("frozen.{label}.opaque"),
                    self.meshes.map_or(0, |m| m.draw_stats().triangles),
                )?;
            }
            self.renderer.encode_plume(
                &mut encoder,
                self.display.scene_color(&self.target.color),
                &self.target.depth,
            );
            self.sky.encode(
                self.backend,
                &mut encoder,
                self.display.scene_color(&self.target.color),
                &self.target.depth,
            );
            self.display.encode(&mut encoder, &self.target.color);
            if let Some(gpu) = &ambient_gpu {
                gpu.resolve(&mut encoder);
            }
            self.backend.queue.submit([encoder.finish()]);
            let pixels = self.target.read_rgba8(self.backend)?;
            let radiance = thessa_render_wgpu::readback::read_color(self.backend, hdr)?;
            let depth =
                thessa_render_wgpu::readback::read_depth(self.backend, &self.target.depth_texture)?;
            if let Some(expected) = &reference_depth {
                if &depth != expected {
                    return Err(format!("frozen depth changed in {label}").into());
                }
            } else {
                std::fs::write(directory.join("depth.f32"), &depth)?;
                reference_depth = Some(depth);
            }
            if mode == V::None {
                let case = usize::from(ambient_enabled);
                if let Some(ambient) = &self.ambient_visibility {
                    let receiver =
                        thessa_render_wgpu::readback::read_color(self.backend, &ambient.texture)?;
                    if reference_ambient.as_ref().is_some_and(|r| r != &receiver) {
                        return Err("frozen diffuse-sky receiver changed in Off/On/Off".into());
                    }
                    reference_ambient = Some(receiver.clone());
                    std::fs::write(
                        directory.join(format!("{label}.diffuse-sky.rgba16f")),
                        receiver,
                    )?;
                }
                if let Some(expected) = &reference_hdr[case] {
                    if &radiance != expected || reference_png[case].as_ref() != Some(&pixels) {
                        return Err("frozen beauty A/B/A is not byte-identical".into());
                    }
                } else {
                    reference_hdr[case] = Some(radiance.clone());
                    reference_png[case] = Some(pixels.clone());
                }
            }
            crate::write_rgba_png(
                &directory.join(format!("{label}.png")),
                size.width,
                size.height,
                &pixels,
            )?;
            std::fs::write(directory.join(format!("{label}.rgba16f")), &radiance)?;
            let ambient_gpu_ms = ambient_gpu
                .as_ref()
                .map(|gpu| gpu.read(self.backend).map(|v| v[0] * 1000.0))
                .transpose()?;
            manifest["views"]
                .as_array_mut()
                .unwrap()
                .push(serde_json::json!({"name":label,"mode":mode,"depth_identical":true,"ambient_visibility_enabled":ambient_enabled,"ambient_visibility_gpu_ms":ambient_gpu_ms}));
            std::fs::write(&manifest_path, serde_json::to_vec_pretty(&manifest)?)?;
        }
        if self.scene.temporal_epoch(self.renderer, self.meshes) != epoch
            || vertices(self.renderer, self.backend)? != geometry
            || self.renderer.read_topology_records(self.backend)? != topology
            || self.renderer.read_topology_state(self.backend)? != topology_state
            || self.renderer.read_draw(self.backend)? != draw
        {
            return Err("published geometry/topology/epoch changed during frozen sequence".into());
        }
        self.renderer
            .set_terrain_debug_view(self.backend, self.settings)?;
        manifest["complete"] = true.into();
        manifest["beauty_round_trip_identical"] = true.into();
        manifest["published_geometry_identical"] = true.into();
        manifest["ambient_receiver_identical"] = if reference_ambient.is_some() {
            true.into()
        } else {
            serde_json::Value::Null
        };
        std::fs::write(&manifest_path, serde_json::to_vec_pretty(&manifest)?)?;
        println!(
            "frozen_terrain_diagnostics={} views={view_count} depth_identical=true beauty_round_trip_identical=true published_geometry_identical=true; diagnostic replay, not production timing",
            directory.display()
        );
        Ok(())
    }
}
