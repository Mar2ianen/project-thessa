//! Basic water reflections: procedural sky cubemap as image-based specular.
//!
//! Ocean pixels are smooth (roughness ~0.16) but had nothing to reflect —
//! the PBR specular lobe sampled no environment, so lakes rendered matte.
//! Each terrain tile gets an [`EnvironmentMapLight`] sharing one small sky
//! cubemap: smooth pixels pick up sky sheen and a sun glint, rough land is
//! unaffected. The diffuse handle is near-black on purpose: land lighting
//! stays exactly as tuned (direct sun only), only specular gains a source.
//!
//! This is the raster baseline. Under Solari the ray-traced specular takes
//! over where rays hit; the cubemap remains a valid fallback. Animated wave
//! normals and water physics (buoyancy/splashdown) are roadmap items, not
//! part of this slice.

use super::*;
use bevy::{
    asset::RenderAssetUsages,
    image::Image,
    light::{EnvironmentMapLight, LightProbe},
    render::render_resource::{
        Extent3d, TextureDataOrder, TextureDimension, TextureFormat, TextureViewDescriptor,
        TextureViewDimension,
    },
};

use crate::atmosphere::PrimaryStarLight;

/// Cubemap edge length in texels. The sun disk is sub-texel at any sane
/// size, so resolution buys gradient smoothness, not glint sharpness.
const SKY_CUBE_SIZE: u32 = 32;

/// Canonical sky radiance in linear units, matched to the 89k-lux primary
/// sun so specular math lands on the same scale as direct diffuse (no
/// arbitrary gain knob on top). The SKY stays dim LDR (subtle sheen, no
/// land wash); only the SUN disk is hot — glint is a mirror of the sun,
/// not of the sky. Verified by screenshot: uniform-bright sky washes the
/// frame through fresnel, a dim sky with a hot disk does not.
const SKY_ZENITH: [f32; 3] = [0.10, 0.19, 0.38];
const SKY_HORIZON: [f32; 3] = [0.42, 0.52, 0.60];
const SKY_GROUND: [f32; 3] = [0.015, 0.018, 0.015];
const SUN_DISK_RADIANCE: f32 = 200000.0;

/// Shared sky environment for water specular plus the sun direction it was
/// baked from (render space, matching the tile transforms).
#[derive(Resource)]
pub(super) struct WaterSky {
    pub image: Handle<Image>,
    pub diffuse_black: Handle<Image>,
    pub sun_dir: Vec3,
}

pub(super) struct WaterPlugin;

impl Plugin for WaterPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, build_water_sky)
            .add_systems(Update, refresh_water_sky);
    }
}

fn sky_texel(dir: Vec3, sun_dir: Vec3, sun_tint: [f32; 3]) -> [f32; 3] {
    let elevation = dir.y.clamp(-1.0, 1.0);
    let mut rgb = if elevation >= 0.0 {
        let t = (elevation / 0.6).clamp(0.0, 1.0);
        // Smoothstepped gradient without trig: cheap at build, exact always.
        let s = t * t * (3.0 - 2.0 * t);
        [
            SKY_HORIZON[0] + (SKY_ZENITH[0] - SKY_HORIZON[0]) * s,
            SKY_HORIZON[1] + (SKY_ZENITH[1] - SKY_HORIZON[1]) * s,
            SKY_HORIZON[2] + (SKY_ZENITH[2] - SKY_HORIZON[2]) * s,
        ]
    } else {
        SKY_GROUND
    };
    // Sun disk plus a tight glow so the glint lobe always finds a hot
    // texel: the true disk (~0.25 deg) is far below one 2.8-deg texel.
    let alignment = dir.dot(sun_dir).clamp(-1.0, 1.0);
    if alignment > 0.9985 {
        rgb = [SUN_DISK_RADIANCE, SUN_DISK_RADIANCE, SUN_DISK_RADIANCE];
    } else if alignment > 0.99 {
        for (i, c) in rgb.iter_mut().enumerate() {
            *c += sun_tint[i] * SUN_DISK_RADIANCE * 0.02;
        }
    }
    rgb
}

/// Face order for [`TextureDimension::Cube`]: +X, -X, +Y, -Y, +Z, -Z.
fn cube_direction(face: usize, x: u32, y: u32, size: u32) -> Vec3 {
    // Texel centers in [-1, 1]; +Y face looks straight up.
    let u = (x as f32 + 0.5) / size as f32 * 2.0 - 1.0;
    let v = (y as f32 + 0.5) / size as f32 * 2.0 - 1.0;
    match face {
        0 => Vec3::new(1.0, -v, -u),
        1 => Vec3::new(-1.0, -v, u),
        2 => Vec3::new(u, 1.0, v),
        3 => Vec3::new(u, -1.0, -v),
        4 => Vec3::new(u, -v, 1.0),
        _ => Vec3::new(-u, -v, -1.0),
    }
    .normalize()
}

fn bake_sky_cubemap(sun_dir: Vec3, sun_tint: [f32; 3]) -> Vec<u8> {
    // HDR float chain end to end: the PBR shader consumes radiance, so no
    // sRGB round-trip happens here (unlike the LDR tile mipmaps).
    let size = SKY_CUBE_SIZE as usize;
    // Full mip chain: the PBR shader selects a level by roughness, and
    // sampling a missing level yields nothing. Faces of each level are
    // stored contiguously (6 layers per level).
    let mut data = Vec::new();
    let mut level_size = size;
    // Keep the previous level's decoded-linear values for the box filter.
    let mut prev_linear: Option<Vec<[f32; 3]>> = None;
    let mut prev_size = 0usize;
    loop {
        let mut level_linear = vec![[0.0f32; 3]; level_size * level_size * 6];
        for face in 0..6 {
            for y in 0..level_size {
                for x in 0..level_size {
                    let linear = match &prev_linear {
                        None => {
                            let dir = cube_direction(face, x as u32, y as u32, size as u32);
                            let rgb = sky_texel(dir, sun_dir, sun_tint);
                            // Base level stores authored radiance verbatim.
                            for c in rgb.into_iter().chain([1.0]) {
                                data.extend_from_slice(&c.to_le_bytes());
                            }
                            rgb
                        }
                        Some(prev) => {
                            // 2x2 box average in linear space.
                            let mut sum = [0.0f32; 3];
                            for dy in 0..2 {
                                for dx in 0..2 {
                                    let px = (x * 2 + dx).min(prev_size - 1);
                                    let py = (y * 2 + dy).min(prev_size - 1);
                                    let src =
                                        prev[(face * prev_size * prev_size) + py * prev_size + px];
                                    for c in 0..3 {
                                        sum[c] += src[c];
                                    }
                                }
                            }
                            let avg = [sum[0] / 4.0, sum[1] / 4.0, sum[2] / 4.0];
                            for c in avg.into_iter().chain([1.0]) {
                                data.extend_from_slice(&c.to_le_bytes());
                            }
                            avg
                        }
                    };
                    level_linear[(face * level_size * level_size) + y * level_size + x] = linear;
                }
            }
        }
        if level_size == 1 {
            break;
        }
        prev_linear = Some(level_linear);
        prev_size = level_size;
        level_size = (level_size / 2).max(1);
    }
    data
}

fn sky_cubemap_image(sun_dir: Vec3, sun_tint: [f32; 3]) -> Image {
    // `Image::new_uninit` defaults to layer-major storage. The baker above
    // writes every mip level contiguously, so the upload must say mip-major.
    let mut image = Image::new_uninit(
        Extent3d {
            width: SKY_CUBE_SIZE,
            height: SKY_CUBE_SIZE,
            depth_or_array_layers: 6,
        },
        TextureDimension::D2,
        TextureFormat::Rgba32Float,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.data_order = TextureDataOrder::MipMajor;
    image.texture_descriptor.mip_level_count = 6;
    image.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        ..default()
    });
    image.data = Some(bake_sky_cubemap(sun_dir, sun_tint));
    image
}

fn black_diffuse_cubemap_image() -> Image {
    let mut image = Image::new(
        Extent3d {
            width: 1,
            height: 1,
            depth_or_array_layers: 6,
        },
        TextureDimension::D2,
        vec![2u8; 6 * 4],
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        ..default()
    });
    image
}

fn sun_render_dir(light_transform: &Transform) -> Vec3 {
    // Aimed by `aim_light_slot`: NEG_Z is the travel direction, so +Z
    // points back at the sun, in render space like the tile transforms.
    (light_transform.rotation * Vec3::Z).normalize_or_zero()
}

fn build_water_sky(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    sun: Query<&Transform, With<PrimaryStarLight>>,
) {
    let sun_dir = sun.single().map(sun_render_dir).unwrap_or(Vec3::Y);
    // Bevy cubemap convention: D2 with 6 array layers (+X, -X, +Y, -Y,
    // +Z, -Z); the builders make the Cube view explicit for the probe path.
    let image = sky_cubemap_image(sun_dir, [1.0, 0.93, 0.85]);
    let black = black_diffuse_cubemap_image();
    commands.insert_resource(WaterSky {
        image: images.add(image),
        diffuse_black: images.add(black),
        sun_dir,
    });
}

/// Re-bake when the primary sun swung (time warp moves it): 6x32 px on
/// the CPU is microseconds, and bytes upload in place — no realloc.
fn refresh_water_sky(
    sky: Option<ResMut<WaterSky>>,
    mut images: ResMut<Assets<Image>>,
    sun: Query<(&Transform, &DirectionalLight), With<PrimaryStarLight>>,
) {
    let Some(mut sky) = sky else { return };
    let Ok((transform, _)) = sun.single() else {
        return;
    };
    let sun_dir = sun_render_dir(transform);
    if sun_dir.dot(sky.sun_dir).clamp(-1.0, 1.0) > 0.99985 {
        return;
    }
    if let Some(mut image) = images.get_mut(&sky.image) {
        image.data = Some(bake_sky_cubemap(sun_dir, [1.0, 0.93, 0.85]));
    }
    sky.sun_dir = sun_dir;
}

/// Probe bundle constructor for terrain tile spawn: shared sky specular
/// plus black diffuse (land lighting untouched). Built at spawn time so
/// there is no query/insert race with tile despawns.
pub(super) fn tile_water_probe(sky: &WaterSky) -> (EnvironmentMapLight, LightProbe) {
    (
        EnvironmentMapLight {
            diffuse_map: sky.diffuse_black.clone(),
            specular_map: sky.image.clone(),
            // Physical units throughout (dim HDR sky + hot sun disk, no gain
            // knob). Tuned by screenshot, not by constant.
            intensity: 1.0,
            ..default()
        },
        // The parent terrain tile supplies the physical transform. Keeping
        // this marker in the bundle makes it impossible to create a map
        // light that Bevy cannot gather as a reflection probe.
        LightProbe::default(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sky_cubemap_has_full_mip_chain_and_hot_sun() {
        let data = bake_sky_cubemap(Vec3::Y, [1.0, 0.93, 0.85]);
        // 32..1 with 6 faces of RGBA32F.
        let expected = (1024 + 256 + 64 + 16 + 4 + 1) * 6 * 16;
        assert_eq!(data.len(), expected);
        // Base level (+Y face looking straight up) is zenith, not white.
        let zenith = &data[(2 * 1024) * 16..(2 * 1024) * 16 + 12];
        let z = [
            f32::from_le_bytes(zenith[0..4].try_into().unwrap()),
            f32::from_le_bytes(zenith[4..8].try_into().unwrap()),
            f32::from_le_bytes(zenith[8..12].try_into().unwrap()),
        ];
        assert!((z[0] - SKY_ZENITH[0]).abs() < 1e-3, "{z:?}");
        // Sun straight overhead lands a hot disk texel somewhere.
        let hot = data
            .as_chunks::<16>()
            .0
            .iter()
            .filter(|t| f32::from_le_bytes(t[0..4].try_into().unwrap()) > SUN_DISK_RADIANCE * 0.5)
            .count();
        assert!((1..=4).contains(&hot), "hot texels: {hot}");
    }

    #[test]
    fn cube_directions_are_unit_and_cover_all_axes() {
        for face in 0..6 {
            let d = cube_direction(face, 3, 5, 32);
            assert!((d.length() - 1.0).abs() < 1e-6);
        }
        // +Y face center looks straight up.
        let up = cube_direction(2, 16, 16, 32);
        assert!(up.y > 0.99, "{up:?}");
    }

    #[test]
    fn water_images_expose_cube_views_and_mip_major_storage() {
        let sky = sky_cubemap_image(Vec3::Y, [1.0, 0.93, 0.85]);
        assert_eq!(sky.data_order, TextureDataOrder::MipMajor);
        assert_eq!(sky.texture_descriptor.mip_level_count, 6);
        assert_eq!(
            sky.texture_view_descriptor
                .as_ref()
                .and_then(|descriptor| descriptor.dimension),
            Some(TextureViewDimension::Cube)
        );
        let black = black_diffuse_cubemap_image();
        assert_eq!(
            black
                .texture_view_descriptor
                .as_ref()
                .and_then(|descriptor| descriptor.dimension),
            Some(TextureViewDimension::Cube)
        );

        let sky = WaterSky {
            image: Handle::default(),
            diffuse_black: Handle::default(),
            sun_dir: Vec3::Y,
        };
        let (environment, probe) = tile_water_probe(&sky);
        let mut world = World::new();
        world.spawn((environment, probe));
        let mut query = world.query::<(&LightProbe, &EnvironmentMapLight)>();
        assert_eq!(query.iter(&world).count(), 1);
    }
}
