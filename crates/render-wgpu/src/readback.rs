//! Explicit diagnostic color readback. No transfer function or tone mapping.
use crate::{Backend, BackendError, terrain::map_buffer};

/// Tightly packed base-level RGBA8 or RGBA16Float bytes. Call outside measured
/// frames; preserves stored values, including pre-exposed linear HDR channels.
pub fn read_color(backend: &Backend, texture: &wgpu::Texture) -> Result<Vec<u8>, BackendError> {
    let fail = |message: &str| BackendError::Device(message.into());
    let bytes_per_pixel = match texture.format() {
        wgpu::TextureFormat::Rgba8Unorm | wgpu::TextureFormat::Rgba8UnormSrgb => 4,
        wgpu::TextureFormat::Rgba16Float => 8,
        _ => return Err(fail("color readback requires RGBA8 or RGBA16Float")),
    };
    let size = texture.size();
    if texture.dimension() != wgpu::TextureDimension::D2
        || size.depth_or_array_layers != 1
        || texture.sample_count() != 1
        || !texture.usage().contains(wgpu::TextureUsages::COPY_SRC)
    {
        return Err(fail(
            "color readback requires one resolved COPY_SRC 2D image",
        ));
    }
    let row = size
        .width
        .checked_mul(bytes_per_pixel)
        .ok_or_else(|| fail("color row overflow"))?;
    let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let stride = row
        .checked_add(alignment - 1)
        .ok_or_else(|| fail("color stride overflow"))?
        / alignment
        * alignment;
    let bytes = u64::from(stride) * u64::from(size.height);
    if bytes > backend.device.limits().max_buffer_size || usize::try_from(bytes).is_err() {
        return Err(fail("color readback exceeds buffer/address budget"));
    }
    let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("diagnostic-color-readback"),
        size: bytes,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(size.height),
            },
        },
        size,
    );
    backend.queue.submit([encoder.finish()]);
    let padded = map_buffer(backend, &staging)?;
    Ok(padded
        .chunks_exact(stride as usize)
        .flat_map(|row_bytes| row_bytes[..row as usize].iter().copied())
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires a wgpu adapter; bounded 7x3 diagnostic images"]
    fn color_readback_preserves_hdr_and_removes_row_padding() {
        let backend = Backend::bring_up().unwrap();
        for (format, pixel) in [
            (wgpu::TextureFormat::Rgba8Unorm, vec![255, 128, 64, 255]),
            (
                wgpu::TextureFormat::Rgba16Float,
                vec![0, 64, 0, 56, 0, 52, 0, 60],
            ),
        ] {
            let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("readback-contract"),
                size: wgpu::Extent3d {
                    width: 7,
                    height: 3,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
                view_formats: &[],
            });
            let view = texture.create_view(&Default::default());
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &view,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color {
                                r: 2.0,
                                g: 0.5,
                                b: 0.25,
                                a: 1.0,
                            }),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    ..Default::default()
                });
            }
            backend.queue.submit([encoder.finish()]);
            assert_eq!(read_color(&backend, &texture).unwrap(), pixel.repeat(21));
        }
    }
}
