//! Actual raster consumption of the backend-neutral authored height layer.
use glam::DVec3;
use thessa_graphics::{ResolvedGraphicsSettings, TerrainDebugView};
use thessa_rcbt_core::{
    HeightPage,
    surface_sculpt::SurfaceSculpt,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend,
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored"]
fn sparse_sculpt_deforms_existing_height_raster_without_an_extra_pass() {
    let backend = Backend::bring_up().expect("requires a local adapter");
    let radius = 3_200_000.0;
    let key = TileKey::new(4, 22, 1 << 21, 1 << 21).unwrap();
    let anchor = TileAnchor::new(key, radius).unwrap();
    let mut sculpt = SurfaceSculpt::new(radius, 0.01, 64).unwrap();
    sculpt.apply_dab(anchor.normal, 0.30, -0.04).unwrap();
    assert!(
        sculpt.allocated_pages() <= 25,
        "a 60 cm stamp must fit its local 5x5 page neighborhood"
    );
    let settings = ResolvedGraphicsSettings {
        terrain_debug_view: TerrainDebugView::Normal,
        terrain_normal_maps: false,
        shadow_enabled: false,
        specular_reflections: false,
        ..Default::default()
    };
    let center = DVec3::from(anchor.anchor_body_m) + DVec3::from(anchor.normal);
    let eye = center + DVec3::from(anchor.normal) * 0.9;
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y).inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.01, 10000.0),
        world_position_m: eye,
        exposure: 1.0,
    };
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let mut frames = Vec::new();
    for edited in [false, true, false] {
        let heights: Vec<_> = (0..33 * 33)
            .map(|i| {
                let u = (i % 33) as f64 / 32.0;
                let v = (i / 33) as f64 / 32.0;
                let raw = DVec3::from(anchor.raw_center)
                    + DVec3::from(anchor.raw_axis_u) * (2.0 * (u - 0.5) * anchor.half_extent)
                    + DVec3::from(anchor.raw_axis_v) * (2.0 * (v - 0.5) * anchor.half_extent);
                1.0 + if edited {
                    sculpt.sample(raw.normalize().to_array()).unwrap()
                } else {
                    0.0
                }
            })
            .collect();
        let page = HeightPage::bake(&heights, 33, 0.001).unwrap();
        let mut renderer = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &[TerrainPatch {
                anchor,
                page: &page,
            }],
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 1,
                material_layers: 1,
                height_grid_size: 33,
            },
            Some(&settings),
        )
        .unwrap();
        renderer.update_view(&backend, &view).unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        frames.push(target.read_rgba8(&backend).unwrap());
    }
    if let Some(directory) = std::env::var_os("THESSA_TEST_ARTIFACT_DIR") {
        let root = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&root).unwrap();
        for (name, frame) in ["base", "sculpt", "restored"].into_iter().zip(&frames) {
            let file = std::fs::File::create(root.join(format!("sculpt-{name}.png"))).unwrap();
            let mut encoder = png::Encoder::new(file, 65, 65);
            encoder.set_color(png::ColorType::Rgba);
            encoder.set_depth(png::BitDepth::Eight);
            encoder
                .write_header()
                .unwrap()
                .write_image_data(frame)
                .unwrap();
        }
    }
    assert_eq!(
        frames[0], frames[2],
        "restoring the base page must remove all sculpt response"
    );
    let changed = frames[0]
        .as_chunks::<4>()
        .0
        .iter()
        .zip(frames[1].as_chunks::<4>().0.iter())
        .filter(|(a, b)| a[..3].iter().zip(&b[..3]).any(|(a, b)| a.abs_diff(*b) > 8))
        .count();
    assert!(
        changed > 200,
        "real raster must resolve the depression, changed pixels={changed}"
    );
    eprintln!(
        "sparse sculpt GPU: changed_pixels={changed}/4225 source_pages={} source_sample_bytes={}",
        sculpt.allocated_pages(),
        sculpt.allocated_bytes()
    );
}
