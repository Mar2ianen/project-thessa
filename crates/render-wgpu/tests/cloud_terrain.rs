//! Production terrain/cloud receiver regression at the WORLD00 coastal direction.
//! No test-only shader or direct call to the cloud visibility helper is used.
use glam::DVec3;
use thessa_graphics::ResolvedGraphicsSettings;
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    material_page::{CbtMaterialPage, MaterialPages},
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend,
    clouds::CloudBinding,
    environment::EnvironmentBinding,
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};
use wgpu::util::DeviceExt;

const RADIUS_M: f64 = 3_200_000.0;
const EXTENT: u32 = 129;
const ROI: std::ops::Range<usize> = 32..97;
// Rgba8Unorm readback: permit one linear quantization code (1/255), not a
// relative tolerance that would hide black false-positive planet shadows.
const TOLERANCE_U8: u8 = 1;

fn coastal_tile(level: u8) -> (TileAnchor, [f64; 2], DVec3) {
    let lat = 23.565679258_f64.to_radians();
    let lon = 165.138815819_f64.to_radians();
    // Canonical worldgen convention: east-positive longitude, negative z.
    let direction = DVec3::new(lat.cos() * lon.cos(), lat.sin(), -lat.cos() * lon.sin());
    assert!(direction.x < -direction.y.abs().max(direction.z.abs()));
    let side = (1_u32 << level) as f64;
    // Face 1 has raw coordinates (-1, b, a).
    let x = (direction.z / -direction.x + 1.0) * 0.5 * side;
    let y = (direction.y / -direction.x + 1.0) * 0.5 * side;
    let anchor = TileAnchor::new(
        TileKey::new(1, level, x.floor() as u32, y.floor() as u32).unwrap(),
        RADIUS_M,
    )
    .unwrap();
    let uv = [x.fract(), y.fract()];
    assert!((DVec3::from(anchor.project_body_m(uv, 0.0)) - direction * RADIUS_M).length() < 1e-8);
    (anchor, uv, direction)
}

fn scene(
    backend: &Backend,
    anchor: TileAnchor,
    page: &HeightPage,
    target: DVec3,
    clearance_m: f64,
    sun: DVec3,
    illuminance_lux: f32,
) -> (TerrainRenderer, CloudBinding) {
    let mut settings = ResolvedGraphicsSettings {
        specular_reflections: false,
        terrain_normal_maps: false,
        shadow_enabled: false,
        night_lights_enabled: false,
        sky_dome: false,
        clouds_enabled: true,
        clouds_volumetric: true,
        clouds_cast_shadows: true,
        clouds_steps: 24,
        ..Default::default()
    };
    settings.ocean.enabled = false;
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        backend,
        &[TerrainPatch { anchor, page }],
        wgpu::TextureFormat::Rgba8Unorm,
        TerrainCapacity {
            patches: 1,
            height_grid_size: page.grid_size(),
            material_layers: 1,
        },
        Some(&settings),
    )
    .unwrap();
    assert!(terrain.shadows().is_none());
    assert!(terrain.conforming_surface().is_none());
    assert_eq!(terrain.land_environment_samples(), 0);
    assert!(!terrain.sky_enabled());
    assert_eq!(terrain.night_lights_stats().storage_bytes, 0);

    let mut id = 8 + u64::from(anchor.tile.face);
    for bit in (0..anchor.tile.level).rev() {
        id = (id << 2) | u64::from(((anchor.tile.x >> bit) & 1) * 2 + ((anchor.tile.y >> bit) & 1));
    }
    let material = CbtMaterialPage::from_rgba8([255; 4].repeat(128 * 128)).unwrap();
    terrain
        .update_materials(
            backend,
            &[id],
            &MaterialPages::from_iter([(id, (1, material))]),
        )
        .unwrap();
    let sun_record = [sun.x as f32, sun.y as f32, sun.z as f32, 0.0];
    terrain
        .update_lighting(
            backend,
            &SurfaceLighting {
                directions: [sun_record; 3],
                colors_lux: [[illuminance_lux; 4], [0.0; 4], [0.0; 4]],
                ..Default::default()
            },
        )
        .unwrap();
    let eye = target + target.normalize() * clearance_m;
    terrain
        .update_view_for_extent(
            backend,
            &ViewParams {
                world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, target, DVec3::Y)
                    .inverse(),
                world_position_m: eye,
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.7, 1.0, 1.0, 1e6),
                exposure: 0.5,
            },
            [EXTENT; 2],
        )
        .unwrap();

    // An enabled physical 3--6 km deck with nonzero extinction, but exactly
    // transparent weather. Only wind.w changes between paired frames.
    let mut records = [[0.0_f32; 4]; 11];
    records[0] = [eye.x as f32, eye.y as f32, eye.z as f32, RADIUS_M as f32];
    records[1] = [3000.0, 3000.0, 0.002, settings.clouds_steps as f32];
    records[2] = [0.0, 1.0, 4.0, 0.0];
    records[3] = [EXTENT as f32, EXTENT as f32, 0.0, 0.0];
    records[4..7].fill(sun_record);
    records[7] = [illuminance_lux; 4];
    let bytes = records
        .into_iter()
        .flatten()
        .flat_map(f32::to_le_bytes)
        .collect::<Vec<_>>();
    let frame = backend
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("coastal-cloud-receiver-frame"),
            contents: &bytes,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
    let weather = backend.device.create_texture_with_data(
        &backend.queue,
        &wgpu::TextureDescriptor {
            label: Some("coastal-transparent-weather"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        },
        wgpu::util::TextureDataOrder::LayerMajor,
        &[0; 4],
    );
    let clouds = CloudBinding {
        frame,
        weather: weather.create_view(&Default::default()),
        sampler: backend.device.create_sampler(&Default::default()),
    };
    let mut environment = EnvironmentBinding::new(&backend.device);
    environment.set_clouds(clouds.clone());
    terrain.share_environment(backend, Some(&environment));
    (terrain, clouds)
}

fn draw(
    backend: &Backend,
    terrain: &TerrainRenderer,
    clouds: &CloudBinding,
    cast: bool,
) -> Vec<u8> {
    // CloudFrame.wind.w: third vec4, fourth float. Same handles/publication,
    // geometry, lighting and camera; no disabled-cloud or alternate shader path.
    backend
        .queue
        .write_buffer(&clouds.frame, 44, &f32::from(cast).to_le_bytes());
    let target =
        OffscreenTarget::with_format(backend, EXTENT, EXTENT, wgpu::TextureFormat::Rgba8Unorm)
            .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    target.read_rgba8(backend).unwrap()
}

fn roi_rgb(pixels: &[u8]) -> Vec<[u8; 3]> {
    ROI.flat_map(|y| {
        ROI.map(move |x| {
            pixels[(y * EXTENT as usize + x) * 4..][..3]
                .try_into()
                .unwrap()
        })
    })
    .collect()
}

#[test]
#[ignore = "hardware: production WORLD00 zero-height terrain must not acquire a planet shadow from transparent clouds"]
fn transparent_cloud_shadows_preserve_outward_lit_coastal_terrain() {
    let backend = Backend::bring_up().unwrap();
    let (anchor, uv, radial) = coastal_tile(6);
    assert!(uv.into_iter().all(|v| (0.4..0.6).contains(&v)));
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let (terrain, clouds) = scene(
        &backend,
        anchor,
        &page,
        radial * RADIUS_M,
        4000.0,
        radial,
        std::f32::consts::PI,
    );
    // Nadir interior ROI spans ~1.47 km, versus >30 km to tile edges:
    // no clear pixels, silhouettes or downward skirts participate. White
    // Lambert terrain at pi lux and exposure 0.5 should read ~128, unsaturated.
    let unshadowed = roi_rgb(&draw(&backend, &terrain, &clouds, false));
    eprintln!(
        "cloud_terrain_unshadowed roi_min={:?} roi_max={:?}",
        unshadowed.iter().flatten().min(),
        unshadowed.iter().flatten().max()
    );
    assert!(
        unshadowed.iter().flatten().all(|v| (120..=136).contains(v)),
        "invalid fixture: interior ROI is not directly lit terrain"
    );
    let shadowed = roi_rgb(&draw(&backend, &terrain, &clouds, true));
    let max_difference = unshadowed
        .iter()
        .flatten()
        .zip(shadowed.iter().flatten())
        .map(|(&a, &b)| a.abs_diff(b))
        .max()
        .unwrap();
    let changed = unshadowed
        .iter()
        .zip(&shadowed)
        .filter(|(a, b)| {
            a.iter()
                .zip(b.iter())
                .any(|(&a, &b)| a.abs_diff(b) > TOLERANCE_U8)
        })
        .count();
    eprintln!(
        "cloud_terrain adapter={:?} radius_m={RADIUS_M} tile={:?} uv={uv:?} roi_pixels={} max_difference_u8={max_difference} changed_pixels={changed} tolerance_u8={TOLERANCE_U8}",
        backend.adapter.get_info(),
        anchor.tile,
        unshadowed.len()
    );
    assert!(
        max_difference <= TOLERANCE_U8,
        "transparent cloud cast shadows darkened {changed}/{} outward-lit zero-height terrain pixels: max RGB difference {max_difference}/255, allowed {TOLERANCE_U8}/255; f32 body reconstruction/chord sagitta is not an interior receiver",
        unshadowed.len()
    );
}

#[test]
#[ignore = "hardware: production sloped terrain facing a below-horizon sun must retain genuine planet occultation"]
fn transparent_cloud_receiver_keeps_genuine_night_side_planet_shadow() {
    let backend = Backend::bring_up().unwrap();
    let (anchor, uv, radial) = coastal_tile(9);
    // A real nonnegative height slope, not a material-normal override: the
    // surface faces a star 10 degrees below its radial horizon. Without this
    // slope a black Lambert back face could hide accidentally removed shadows.
    let page = HeightPage::bake(&[0.0, 4000.0, 0.0, 4000.0], 2, 0.001).unwrap();
    let axis = DVec3::from(anchor.raw_axis_u);
    let downhill = -(axis - radial * radial.dot(axis)).normalize();
    let depression = 10_f64.to_radians();
    let sun = downhill * depression.cos() - radial * depression.sin();
    let target = radial * (RADIUS_M + f64::from(page.sample(uv[0] as f32, uv[1] as f32)));
    assert!(target.dot(sun) < 0.0);
    // Independent f64 closest approach: even the highest point of this page
    // is behind the planet, with more than 1 km of radial clearance margin.
    assert!((RADIUS_M + 4000.0) * depression.cos() < RADIUS_M - 1000.0);
    let (terrain, clouds) = scene(
        &backend,
        anchor,
        &page,
        target,
        1000.0,
        sun,
        4.0 * std::f32::consts::PI,
    );
    // ROI footprint ~0.37 km; the nearest tile edge is >0.6 km away.
    let unshadowed = roi_rgb(&draw(&backend, &terrain, &clouds, false));
    eprintln!(
        "cloud_terrain_night_unshadowed roi_min={:?} roi_max={:?}",
        unshadowed.iter().flatten().min(),
        unshadowed.iter().flatten().max()
    );
    assert!(
        unshadowed.iter().flatten().all(|&v| v > 20 && v < 240),
        "invalid night fixture: slope must actually face the source without saturating"
    );
    let shadowed = roi_rgb(&draw(&backend, &terrain, &clouds, true));
    let max_shadowed = *shadowed.iter().flatten().max().unwrap();
    eprintln!(
        "cloud_terrain_night adapter={:?} roi_pixels={} max_shadowed_u8={max_shadowed} tolerance_u8={TOLERANCE_U8}",
        backend.adapter.get_info(),
        shadowed.len()
    );
    assert!(
        max_shadowed <= TOLERANCE_U8,
        "genuine below-horizon planet shadow disappeared: max RGB {max_shadowed}/255"
    );
}
