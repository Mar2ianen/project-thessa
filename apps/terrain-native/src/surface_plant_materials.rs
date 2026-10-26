//! Imported CC0 optical patches, loaded only by enabled plant publication.
use std::sync::OnceLock;
use thessa_render_core::mesh::{Image, MeshAsset, Texture, Wrap};

const LEAF_WIDTH: u32 = 128;
const LEAF_HEIGHT: u32 = 48;
const GUTTER: u32 = 16;

pub fn foliage_uv(uv: [f32; 2], tone: u8) -> [f32; 2] {
    let width = LEAF_WIDTH + 2 * GUTTER;
    let height = LEAF_HEIGHT + 2 * GUTTER;
    [
        (GUTTER as f32 + 0.5 + uv[0] * (LEAF_WIDTH - 1) as f32) / width as f32,
        (f32::from(tone.min(3)) * height as f32
            + GUTTER as f32
            + 0.5
            + uv[1] * (LEAF_HEIGHT - 1) as f32)
            / (4 * height) as f32,
    ]
}

fn foliage_atlas(source: Image, map: usize) -> Image {
    assert_eq!((source.width, source.height), (LEAF_WIDTH, LEAF_HEIGHT));
    let width = source.width + 2 * GUTTER;
    let tile_height = source.height + 2 * GUTTER;
    let mut rgba8 = Vec::with_capacity((width * tile_height * 4 * 4) as usize);
    // Authored linear reflectance grades, not measured chlorophyll spectra.
    let grades = [
        [0.82, 0.94, 0.86],
        [1.32, 1.35, 0.92],
        [1.22, 1.03, 0.67],
        [1.70, 0.65, 0.35],
    ];
    // Authored cuticle roughness: mature, emerging, olive and weathered.
    let roughness = [0.70, 0.50, 0.80, 0.90];
    for (tone, grade) in grades.into_iter().enumerate() {
        for y in 0..tile_height {
            for x in 0..width {
                let sx = x.saturating_sub(GUTTER).min(source.width - 1);
                let sy = y.saturating_sub(GUTTER).min(source.height - 1);
                let offset = ((sy * source.width + sx) * 4) as usize;
                for (c, &gain) in grade.iter().enumerate() {
                    let source_value = source.rgba8[offset + c];
                    let value = if map == 0 {
                        let encoded = f32::from(source_value) / 255.0;
                        let linear = if encoded <= 0.04045 {
                            encoded / 12.92
                        } else {
                            ((encoded + 0.055) / 1.055).powf(2.4)
                        };
                        let value = (linear * gain).clamp(0.0, 1.0);
                        let encoded = if value <= 0.0031308 {
                            value * 12.92
                        } else {
                            1.055 * value.powf(1.0 / 2.4) - 0.055
                        };
                        (encoded * 255.0).round() as u8
                    } else if map == 2 && c == 1 {
                        (f32::from(source_value) * roughness[tone]).round() as u8
                    } else {
                        source_value
                    };
                    rgba8.push(value);
                }
                rgba8.push(source.rgba8[offset + 3]);
            }
        }
    }
    Image {
        width,
        height: tile_height * 4,
        rgba8,
    }
}

pub fn append(asset: &mut MeshAsset, stem: bool) -> [usize; 3] {
    static MAPS: OnceLock<Vec<Image>> = OnceLock::new();
    let maps = MAPS.get_or_init(|| {
        [
            include_bytes!("../../../assets/textures/plants/bark-base.png").as_slice(),
            include_bytes!("../../../assets/textures/plants/bark-normal.png").as_slice(),
            include_bytes!("../../../assets/textures/plants/bark-mr.png").as_slice(),
            include_bytes!("../../../assets/textures/plants/pinna-base.png").as_slice(),
            include_bytes!("../../../assets/textures/plants/pinna-normal.png").as_slice(),
            include_bytes!("../../../assets/textures/plants/pinna-mr.png").as_slice(),
        ]
        .into_iter()
        .enumerate()
        .map(|(index, bytes)| {
            let mut reader = png::Decoder::new(std::io::Cursor::new(bytes))
                .read_info()
                .expect("bundled optical PNG");
            let mut data = vec![0; reader.output_buffer_size().expect("bounded optical PNG")];
            let info = reader
                .next_frame(&mut data)
                .expect("bundled optical pixels");
            assert_eq!(info.color_type, png::ColorType::Rgba);
            data.truncate(info.buffer_size());
            let image = Image {
                width: info.width,
                height: info.height,
                rgba8: data,
            };
            if index >= 3 {
                foliage_atlas(image, index - 3)
            } else {
                image
            }
        })
        .collect()
    });
    let offset = if stem { 0 } else { 3 };
    std::array::from_fn(|i| {
        let source = &maps[offset + i];
        let image = asset
            .images
            .iter()
            .position(|image| {
                image.width == source.width
                    && image.height == source.height
                    && image.rgba8 == source.rgba8
            })
            .unwrap_or_else(|| {
                let index = asset.images.len();
                asset.images.push(source.clone());
                index
            });
        let wrap = if stem {
            [Wrap::Repeat; 2]
        } else {
            [Wrap::Clamp; 2]
        };
        if let Some(index) = asset.textures.iter().position(|texture| {
            texture.image == image && texture.wrap == wrap && texture.linear && texture.min_linear
        }) {
            return index;
        }
        let index = asset.textures.len();
        asset.textures.push(Texture {
            image,
            wrap,
            linear: true,
            min_linear: true,
        });
        index
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn atlas_preserves_normals_and_filters_pigment_and_roughness_cohorts() {
        let mut asset = MeshAsset {
            primitives: vec![],
            materials: vec![],
            images: vec![],
            textures: vec![],
        };
        let textures = append(&mut asset, false);
        assert_eq!(textures, append(&mut asset, false));
        assert_eq!((asset.images.len(), asset.textures.len()), (3, 3));
        for (map, texture) in textures.into_iter().enumerate() {
            let image = &asset.images[asset.textures[texture].image];
            assert_eq!((image.width, image.height), (160, 320));
            let levels = image.mip_chain(map == 0).unwrap();
            for level in levels.iter().take(5) {
                let pixels: Vec<_> = (0..4)
                    .map(|tone| {
                        let uv = foliage_uv([0.5; 2], tone);
                        let x = (uv[0] * level.width as f32) as u32;
                        let y = (uv[1] * level.height as f32) as u32;
                        let offset = ((y * level.width + x) * 4) as usize;
                        &level.rgba8[offset..offset + 4]
                    })
                    .collect();
                if map == 0 {
                    assert!(pixels.windows(2).all(|pair| pair[0] != pair[1]));
                } else if map == 1 {
                    assert!(pixels.windows(2).all(|pair| pair[0] == pair[1]));
                } else {
                    for (pixel, expected) in pixels.iter().zip([179_i16, 128, 204, 230]) {
                        assert!((i16::from(pixel[1]) - expected).abs() <= 1);
                        assert_eq!([pixel[0], pixel[2], pixel[3]], [255, 0, 255]);
                    }
                    assert!(pixels[1][1] < pixels[0][1]);
                    assert!(pixels[0][1] < pixels[2][1]);
                    assert!(pixels[2][1] < pixels[3][1]);
                }
            }
            for tone in 0..4 {
                let lo = foliage_uv([0.0; 2], tone);
                let hi = foliage_uv([1.0; 2], tone);
                assert!(lo[0] > 0.0 && hi[0] < 1.0);
                assert!(lo[1] > f32::from(tone) * 0.25);
                assert!(hi[1] < f32::from(tone + 1) * 0.25);
            }
        }
        let image = &asset.images[asset.textures[textures[0]].image];
        let means: [[f32; 3]; 4] = std::array::from_fn(|tone| {
            let mut mean = [0.0; 3];
            for y in 0..LEAF_HEIGHT {
                for x in 0..LEAF_WIDTH {
                    let row = tone as u32 * (LEAF_HEIGHT + 2 * GUTTER) + GUTTER + y;
                    let pixel = ((row * image.width + GUTTER + x) * 4) as usize;
                    for (c, value) in mean.iter_mut().enumerate() {
                        let s = f32::from(image.rgba8[pixel + c]) / 255.0;
                        let linear = if s <= 0.04045 {
                            s / 12.92
                        } else {
                            ((s + 0.055) / 1.055).powf(2.4)
                        };
                        *value +=
                            linear * [0.31, 0.35, 0.19][c] / (LEAF_WIDTH * LEAF_HEIGHT) as f32;
                    }
                }
            }
            mean
        });
        assert!(means[0][1] > means[0][0]);
        let luminance = |rgb: [f32; 3]| rgb[0] * 0.2126 + rgb[1] * 0.7152 + rgb[2] * 0.0722;
        assert!(luminance(means[1]) > luminance(means[0]) * 1.15);
        assert!(means[2][0] / means[2][1] > means[0][0] / means[0][1] * 1.2);
        assert!(means[3][0] > means[3][1] * 1.2);
    }
}
