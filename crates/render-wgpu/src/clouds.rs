//! Shared GPU cloud publication. The caller owns weather, SI deck parameters and
//! simulation phase; this binding transports them to all native consumers.
#[derive(Clone)]
pub struct CloudBinding {
    pub frame: wgpu::Buffer,
    pub weather: wgpu::TextureView,
    pub sampler: wgpu::Sampler,
}
pub const SHADER: &str = include_str!("clouds.wgsl");
impl CloudBinding {
    pub fn disabled(device: &wgpu::Device) -> Self {
        let frame = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("disabled-cloud-frame"),
            size: 176,
            usage: wgpu::BufferUsages::UNIFORM,
            mapped_at_creation: false,
        });
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("disabled-cloud-weather"),
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
        });
        Self {
            frame,
            weather: texture.create_view(&Default::default()),
            sampler: device.create_sampler(&Default::default()),
        }
    }
    pub fn entries(&self) -> [wgpu::BindGroupEntry<'_>; 3] {
        [
            wgpu::BindGroupEntry {
                binding: 5,
                resource: self.frame.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 6,
                resource: wgpu::BindingResource::TextureView(&self.weather),
            },
            wgpu::BindGroupEntry {
                binding: 7,
                resource: wgpu::BindingResource::Sampler(&self.sampler),
            },
        ]
    }
}
