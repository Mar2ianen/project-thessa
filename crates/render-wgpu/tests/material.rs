use glam::DVec3;
use thessa_rcbt_core::{
    HeightPage,
    material_page::{CbtMaterialPage, MaterialPages},
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend, BackendError,
    terrain::{OffscreenTarget, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

#[test]
fn material_publication_reuses_uploads_falls_back_and_rejects_invalid_snapshots() {
    let backend = match Backend::bring_up() {
        Ok(b) => b,
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no GPU");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    let anchor = TileAnchor::new(TileKey::new(4, 10, 512, 512).unwrap(), 6_371_000.0).unwrap();
    let page = HeightPage::bake(&[120.0; 4], 2, 0.01).unwrap();
    let mut renderer = TerrainRenderer::with_material_capacity(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
        2,
    )
    .unwrap();
    let center = DVec3::from(anchor.anchor_body_m);
    let eye = center + DVec3::from(anchor.normal) * 20_000.0;
    renderer
        .update_view(
            &backend,
            &ViewParams {
                world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y)
                    .inverse(),
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 1.0, 1e6),
                world_position_m: eye,
                exposure: 1.0,
            },
        )
        .unwrap();
    let target = OffscreenTarget::new(&backend, 257, 257).unwrap();
    let render = |renderer: &TerrainRenderer| {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        target.read_rgba8(&backend).unwrap()
    };
    let fallback = render(&renderer);
    let red = CbtMaterialPage::from_rgba8([255, 0, 0, 255].repeat(128 * 128)).unwrap();
    let blue = CbtMaterialPage::from_rgba8([0, 0, 255, 255].repeat(128 * 128)).unwrap();
    let mut pages = MaterialPages::from([(12, (1, red.clone()))]);
    let stats = renderer.update_materials(&backend, &[12], &pages).unwrap();
    assert_eq!(
        (stats.pages, stats.bytes, stats.residents),
        (1, red.byte_len(), 1)
    );
    let red_frame = render(&renderer);
    let mid = (128 * 257 + 128) * 4;
    assert!(red_frame[mid] > red_frame[mid + 2] + 30);
    assert_ne!(
        fallback, red_frame,
        "resident parent must affect actual raster"
    );
    assert_eq!(
        renderer
            .update_materials(&backend, &[12, 12], &pages)
            .unwrap()
            .pages,
        0
    );
    assert_eq!(render(&renderer), red_frame);
    // Invalid IDs and incomplete decoded mips must not mutate previous state.
    pages.insert(16, (2, blue.clone()));
    assert!(renderer.update_materials(&backend, &[12], &pages).is_err());
    assert_eq!(render(&renderer), red_frame);
    pages.remove(&16);
    pages.insert(
        12,
        (
            2,
            CbtMaterialPage::from_decoded_mips(vec![vec![0; 128 * 128 * 4]]).unwrap(),
        ),
    );
    assert!(renderer.update_materials(&backend, &[12], &pages).is_err());
    assert_eq!(render(&renderer), red_frame);
    pages.insert(12, (3, blue));
    assert_eq!(
        renderer
            .update_materials(&backend, &[12], &pages)
            .unwrap()
            .pages,
        1
    );
    let blue_frame = render(&renderer);
    assert!(blue_frame[mid + 2] > blue_frame[mid] + 30);
    // Removal invalidates the GPU directory even though the layer stays warm.
    pages.clear();
    let stats = renderer.update_materials(&backend, &[], &pages).unwrap();
    assert_eq!((stats.pages, stats.residents), (0, 0));
    assert_eq!(render(&renderer), fallback);
    pages.insert(12, (3, red)); // same generation/payload identity contract required
    pages.get_mut(&12).unwrap().0 = 4;
    assert_eq!(
        renderer
            .update_materials(&backend, &[12], &pages)
            .unwrap()
            .pages,
        1
    );
    assert_eq!(render(&renderer), red_frame);
}
