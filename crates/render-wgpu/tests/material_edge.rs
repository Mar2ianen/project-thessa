use glam::DVec3;
use thessa_rcbt_core::{
    HeightPage,
    material_page::{CbtMaterialPage, MaterialPages},
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend,
    terrain::{OffscreenTarget, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

const LEVEL: u8 = 25;
const GEOMETRY_LEVEL: u8 = 22;
const RADIUS_M: f64 = 3_200_000.0;
const SIZE: u32 = 65;
const FOOTPRINT_M: f64 = 0.000_734;

fn deep_material_id(tile: TileKey) -> u64 {
    if tile.level <= GEOMETRY_LEVEL {
        return thessa_rcbt_core::conforming_surface::page_id(tile).unwrap();
    }
    let depth = tile.level - GEOMETRY_LEVEL;
    let geometry =
        TileKey::new(tile.face, GEOMETRY_LEVEL, tile.x >> depth, tile.y >> depth).unwrap();
    let mut id = thessa_rcbt_core::conforming_surface::page_id(geometry).unwrap();
    for bit in (0..depth).rev() {
        let pair = (((tile.x >> bit) & 1) << 1) | ((tile.y >> bit) & 1);
        id = (id << 2) | u64::from(pair);
    }
    id
}

/// Project a body direction into the independent CPU cube chart and quantize
/// the result to a material tile. The input is generated outside the current
/// tile with TileAnchor's f64 surface projection, not from shader UV math.
fn tile_for_direction(direction: DVec3, level: u8) -> TileKey {
    let direction = direction.normalize();
    let [x, y, z] = direction.to_array();
    let [ax, ay, az] = [x.abs(), y.abs(), z.abs()];
    let (face, u, v, major) = if ax >= ay && ax >= az {
        if x >= 0.0 {
            (0, -z, y, ax)
        } else {
            (1, z, y, ax)
        }
    } else if ay >= az {
        if y >= 0.0 {
            (2, x, -z, ay)
        } else {
            (3, x, z, ay)
        }
    } else if z >= 0.0 {
        (4, x, y, az)
    } else {
        (5, -x, y, az)
    };
    let extent = 1_u32 << level;
    let coordinate = |value: f64| {
        let uv = (value / major * 0.5 + 0.5).clamp(0.0, 1.0);
        ((uv * f64::from(extent)).floor() as u32).min(extent - 1)
    };
    TileKey::new(face, level, coordinate(u), coordinate(v)).unwrap()
}

fn adjacent_tile(current: TileKey, edge: usize) -> TileKey {
    let extent = 1_u32 << current.level;
    let anchor = TileAnchor::new(current, RADIUS_M).unwrap();
    let uv = match edge {
        0 => [-0.5, 0.5],
        1 => [1.5, 0.5],
        2 => [0.5, -0.5],
        _ => [0.5, 1.5],
    };
    let outside = DVec3::from(anchor.project_body_m(uv, 0.0));
    let tile = tile_for_direction(outside, current.level);
    assert_ne!(tile.face, current.face, "edge={edge} must change cube face");
    assert_eq!(tile.level, current.level);
    assert!(tile.x < extent && tile.y < extent);
    tile
}

#[test]
#[ignore = "requires a wgpu adapter; cross-face material residency feather oracle"]
fn l25_cross_face_neighbor_pages_keep_resident_edge_coverage() {
    let backend = Backend::bring_up().expect("requires the local hardware adapter");
    let footprint_fov = 2.0 * (FOOTPRINT_M * f64::from(SIZE) / 4.0).atan();
    let height = HeightPage::bake(&[0.0; 4], 2, 0.01).unwrap();
    let material = CbtMaterialPage::from_rgba8([180, 72, 48, 255].repeat(128 * 128)).unwrap();
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        surface_quality: thessa_graphics::Quality::High,
        terrain_debug_view: thessa_graphics::TerrainDebugView::ResolvedMaterialLod,
        terrain_material_anisotropy: 1,
        terrain_normal_maps: false,
        terrain_ground_detail: true,
        specular_reflections: false,
        shadow_enabled: false,
        ..Default::default()
    };
    assert_eq!(settings.terrain_material_max_level(), LEVEL);
    let target = OffscreenTarget::new(&backend, SIZE, SIZE).unwrap();

    for level in [17, 24, LEVEL] {
        for face in 0..6 {
            for edge in 0..4 {
                let extent = 1_u32 << level;
                let geometry_level = level.min(GEOMETRY_LEVEL);
                let other = extent / 2 + 37;
                let current = TileKey::new(
                    face,
                    level,
                    if edge == 0 {
                        0
                    } else if edge == 1 {
                        extent - 1
                    } else {
                        other
                    },
                    if edge == 2 {
                        0
                    } else if edge == 3 {
                        extent - 1
                    } else {
                        other
                    },
                )
                .unwrap();
                let adjacent = adjacent_tile(current, edge);
                let current_id = deep_material_id(current);
                let adjacent_id = deep_material_id(adjacent);
                let geometry = TileKey::new(
                    current.face,
                    geometry_level,
                    current.x >> (level - geometry_level),
                    current.y >> (level - geometry_level),
                )
                .unwrap();
                let geometry_anchor = TileAnchor::new(geometry, RADIUS_M).unwrap();
                let current_anchor = TileAnchor::new(current, RADIUS_M).unwrap();
                let local = match edge {
                    0 => [0.01, 0.5],
                    1 => [0.99, 0.5],
                    2 => [0.5, 0.01],
                    _ => [0.5, 0.99],
                };
                let center = DVec3::from(current_anchor.project_body_m(local, 0.0));
                let eye = center + DVec3::from(current_anchor.normal) * 2.0;
                let view = ViewParams {
                    world_from_view: glam::dcamera::rh::view::look_at_mat4(
                        eye,
                        center,
                        DVec3::from(current_anchor.raw_axis_v),
                    )
                    .inverse(),
                    clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                        footprint_fov,
                        1.0,
                        0.01,
                        1000.0,
                    ),
                    world_position_m: eye,
                    exposure: 1.0,
                };
                let mut renderer = TerrainRenderer::with_capacity_and_lighting(
                    &backend,
                    &[TerrainPatch {
                        anchor: geometry_anchor,
                        page: &height,
                    }],
                    OffscreenTarget::FORMAT,
                    thessa_render_wgpu::terrain::TerrainCapacity {
                        patches: 1,
                        material_layers: 2,
                        height_grid_size: 2,
                    },
                    Some(&settings),
                )
                .unwrap();
                renderer.update_view(&backend, &view).unwrap();
                renderer
                    .update_lighting(
                        &backend,
                        &thessa_rcbt_core::lighting::SurfaceLighting {
                            ambient_lux: [std::f32::consts::PI; 4],
                            directions: [[0.0; 4]; 3],
                            colors_lux: [[0.0; 4]; 3],
                        },
                    )
                    .unwrap();

                let capture_coverage = |renderer: &TerrainRenderer| {
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    renderer.encode(&mut encoder, &target.color, &target.depth);
                    backend.queue.submit([encoder.finish()]);
                    target.read_rgba8(&backend).unwrap()[(32 * SIZE as usize + 32) * 4 + 1]
                };
                let with_neighbor: MaterialPages = [
                    (current_id, (1, material.clone())),
                    (adjacent_id, (1, material.clone())),
                ]
                .into_iter()
                .collect();
                renderer
                    .update_materials(&backend, &[current_id, adjacent_id], &with_neighbor)
                    .unwrap();
                let covered = capture_coverage(&renderer);
                assert!(
                    covered > 250,
                    "level={level} face={face} edge={edge} adjacent={adjacent:?} resident coverage should be full, got {covered}"
                );

                let current_only: MaterialPages =
                    [(current_id, (1, material.clone()))].into_iter().collect();
                renderer
                    .update_materials(&backend, &[current_id], &current_only)
                    .unwrap();
                let feathered = capture_coverage(&renderer);
                assert!(
                    feathered > 0 && feathered < 230,
                    "level={level} face={face} edge={edge} absent neighbor should feather current page, got {feathered}"
                );
            }
        }
    }
}
