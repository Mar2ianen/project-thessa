//! Imported CC0 optical patches, loaded only by enabled plant publication.
use std::sync::OnceLock;
use thessa_render_core::mesh::{Image, MeshAsset, Texture, Wrap};

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
        .map(|bytes| {
            let mut reader = png::Decoder::new(std::io::Cursor::new(bytes))
                .read_info()
                .expect("bundled optical PNG");
            let mut data = vec![0; reader.output_buffer_size().expect("bounded optical PNG")];
            let info = reader
                .next_frame(&mut data)
                .expect("bundled optical pixels");
            assert_eq!(info.color_type, png::ColorType::Rgba);
            data.truncate(info.buffer_size());
            Image {
                width: info.width,
                height: info.height,
                rgba8: data,
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
