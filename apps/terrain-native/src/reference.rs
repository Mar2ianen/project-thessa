//! Native reference adapter: existing mesh/display/perf consumers, no game Scene.
use std::{error::Error, path::Path, time::Instant};
use thessa_graphics::RequestedGraphics;
use thessa_render_wgpu::{
    Backend,
    display::DisplayTransform,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::OffscreenTarget,
    timing::GpuIntervals,
};

fn validate_baseline(requested: &RequestedGraphics) -> Result<(), &'static str> {
    if requested.renderer.ray_tracing != thessa_graphics::RayTracingRequest::Off
        || requested.shadows.enabled
        || requested.renderer.msaa_samples != 1
        || requested.temporal.taa
        || requested.temporal.motion_vectors
        || requested.temporal.jitter
        || requested.renderer.auto_exposure
        || requested.renderer.bloom
        || !requested.renderer.specular_reflections
        || requested.renderer.resolution_scale != 1.0
    {
        return Err(
            "The material labs require an explicit raster baseline: RT/shadows/temporal/bloom/auto exposure off, specular response on, MSAA=1 and resolution_scale=1; use the matching b01-graphics.toml or b05-graphics.toml",
        );
    }
    Ok(())
}

fn validate_output_paths(paths: &[&Path]) -> Result<(), Box<dyn Error>> {
    let mut unique = std::collections::BTreeSet::new();
    for path in paths {
        let parent = path
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        let file = path
            .file_name()
            .ok_or("capture output requires a filename")?;
        if !unique.insert(parent.canonicalize()?.join(file)) {
            return Err(
                "reference image, perf, HDR and scene sidecars require distinct paths".into(),
            );
        }
    }
    Ok(())
}

pub fn material_lab(
    requested: &RequestedGraphics,
    image: &Path,
    perf_path: &Path,
    linear_hdr: Option<&Path>,
    extent: [u32; 2],
    frames: u32,
    case: &str,
) -> Result<(), Box<dyn Error>> {
    validate_baseline(requested)?;
    if !(1..=1800).contains(&frames) {
        return Err("reference capture requires 1..1800 measured frames".into());
    }
    let directory = image
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let files = [
        directory.join("scene.glb"),
        directory.join("scene.json"),
        directory.join("scene.LICENSE.txt"),
    ];
    let csv_path = perf_path.with_extension("csv");
    let mut outputs = vec![image, perf_path, csv_path.as_path()];
    outputs.extend(files.iter().map(|p| p.as_path()));
    outputs.extend(linear_hdr);
    validate_output_paths(&outputs)?;
    // Sidecar provenance must never overwrite another scene's inputs.
    if files.iter().any(|p| p.exists()) {
        return Err("reference scene sidecars already exist; choose a new output directory".into());
    }
    let scene = match case {
        "b01" => thessa_reference_scenes::MaterialLab::new(),
        "b05" => thessa_reference_scenes::MaterialLab::transparency(),
        _ => return Err(format!("unknown material reference scene: {case}").into()),
    };
    let description = serde_json::to_vec_pretty(&scene.description())?;
    for (path, bytes) in [
        (&files[0], scene.glb()),
        (&files[1], description.as_slice()),
        (
            &files[2],
            thessa_reference_scenes::GEOMETRY_LICENSE_NOTICE.as_bytes(),
        ),
    ] {
        use std::io::Write;
        std::fs::OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(path)?
            .write_all(bytes)?;
    }
    // Exercise the same portable GLB consumed by external renderers, including
    // its material bindings; no private showcase mesh replaces the dataset.
    let asset = if case == "b01" {
        thessa_render_assets::decode_glb(scene.glb(), Default::default())?
    } else {
        thessa_render_assets::decode_glb_surfaces(scene.glb(), Default::default())?
    };
    let backend = Backend::bring_up_for_graphics_profiled(requested)?;
    let resolved = backend.resolve_graphics(requested);
    let target = OffscreenTarget::new(&backend, extent[0], extent[1])?;
    let display = DisplayTransform::new(
        &backend,
        &resolved,
        OffscreenTarget::FORMAT,
        extent[0],
        extent[1],
    )?;
    let mut meshes = MeshRenderer::with_surface(
        &backend,
        &asset,
        SurfaceSettings {
            format: display.scene_format(),
            capacity: 1,
            quality: resolved.surface_quality,
            reflections: resolved.specular_reflections,
            shadows: None,
            ray_tracing: Some(&resolved),
        },
    )?;
    let instance = thessa_render_core::mesh::MeshInstance {
        position_m: glam::DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
        scale: 1.0,
    };
    let view = scene
        .view(extent, crate::manual_exposure(resolved.exposure_ev100))
        .ok_or("invalid reference camera")?;
    let mut light = thessa_rcbt_core::lighting::SurfaceLighting::default();
    light.directions[0][..3].copy_from_slice(&scene.direction_to_light());
    light.colors_lux[0][..3].copy_from_slice(&scene.illuminance_rgb_lux());
    let gpu = GpuIntervals::new(&backend, 4);
    let mut collector = thessa_perf::PerfCollector::with_default_capacity();
    for frame in 0..frames + 30 {
        let started = Instant::now();
        meshes.update(&backend, &view, &light, &[instance])?;
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        if let Some(gpu) = &gpu {
            gpu.mark(&mut encoder, 0);
        }
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: display.scene_color(&target.color),
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
        }
        if let Some(gpu) = &gpu {
            gpu.mark(&mut encoder, 1);
        }
        meshes.encode(
            &mut encoder,
            display.scene_color(&target.color),
            &target.depth,
        );
        if let Some(gpu) = &gpu {
            gpu.mark(&mut encoder, 2);
        }
        display.encode(&mut encoder, &target.color);
        if let Some(gpu) = &gpu {
            gpu.mark(&mut encoder, 3);
            gpu.resolve(&mut encoder);
        }
        backend.queue.submit([encoder.finish()]);
        let cpu = started.elapsed().as_secs_f64();
        backend.device.poll(wgpu::PollType::wait_indefinitely())?;
        let wall = started.elapsed().as_secs_f64();
        if frame >= 30 {
            collector.begin_frame();
            collector.record_cpu_scope("frame.cpu", cpu);
            collector.set_gpu_frame(crate::perf::gpu_frame(
                gpu.as_ref(),
                &backend,
                &[
                    "render.clear",
                    "render.mesh_raster",
                    "render.display_transform",
                ],
            )?);
            collector.end_frame(wall, cpu, 0.0);
        }
    }
    crate::write_rgba_png(image, extent[0], extent[1], &target.read_rgba8(&backend)?)?;
    if let (Some(path), Some(texture)) = (linear_hdr, display.scene_texture()) {
        std::fs::write(
            path,
            thessa_render_wgpu::readback::read_color(&backend, texture)?,
        )?;
    }
    let mut metadata = crate::perf::renderer_capture_metadata(&backend, requested);
    metadata.scenario = scene.id().into();
    metadata.display.width = extent[0];
    metadata.display.height = extent[1];
    metadata.display.vsync = false;
    metadata.window_frames = collector.len();
    crate::perf::capture_frame_inputs(&mut metadata, &view, &light, 0.0, display.enabled());
    let stats = meshes.draw_stats();
    for (key,value) in [
        ("gpu_scope_layout","reference-mesh-v1".into()),
        ("measurement_boundary","Fixed generated GLB; 30 warmup; serial frames; frame.cpu through submit, frame.wall through GPU completion, excluding timestamp map/image readback/serialization. Three GPU intervals: clear, mesh raster, display. Static mesh work recorded in header; no terrain, streaming, simulation or UI. Geometry/lighting unchanged across frames; physical/visual correctness not inferred from timings.".into()),
        ("reference_geometry_file","scene.glb".into()),("reference_scene_file","scene.json".into()),
        ("reference_mesh_candidate_triangles",stats.candidate_triangles.to_string()),
        ("reference_mesh_submitted_triangles",stats.triangles.to_string()),
        ("reference_mesh_api_draws",stats.draw_calls.to_string()),
        ("reference_mesh_logical_draws",stats.draw_commands.to_string()),
        ("reference_scene_frozen","true".into()),
        ("reference_unavailable_features","shadows, RT, area lights, environment/HDRI, temporal reconstruction, normal-map/texture variants, physical glass refraction and GI; not part of these fixed material baselines".into()),
    ] {metadata.graphics.resolved.insert(key.into(),value);}
    let capture = collector.snapshot_capture(metadata);
    capture.write_json(perf_path)?;
    capture.write_csv(&perf_path.with_extension("csv"))?;
    eprintln!(
        "reference_scene={} vertices={} submitted_triangles={} measured_frames={frames} gpu_timestamps={}",
        scene.id(),
        asset
            .primitives
            .iter()
            .map(|p| p.vertices.len())
            .sum::<usize>(),
        stats.triangles,
        gpu.is_some()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fixed_lab_profile_is_explicit_and_rejects_hidden_work_changes() {
        let mut requested = RequestedGraphics::from_toml(include_str!(
            "../../../data/reference-scenes/b01-graphics.toml"
        ))
        .unwrap();
        validate_baseline(&requested).unwrap();
        requested.renderer.specular_reflections = false;
        assert!(validate_baseline(&requested).is_err());
        requested.renderer.specular_reflections = true;
        requested.temporal.taa = true;
        assert!(validate_baseline(&requested).is_err());
        requested.temporal.taa = false;
        requested.shadows.enabled = true;
        assert!(validate_baseline(&requested).is_err());
    }
    #[test]
    fn reference_artifacts_cannot_alias_the_image_or_each_other() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let first = root.join("frame.png");
        let alias = root.join("./frame.png");
        let other = root.join("perf.json");
        validate_output_paths(&[&first, &other]).unwrap();
        assert!(validate_output_paths(&[&first, &alias]).is_err());
    }
}
