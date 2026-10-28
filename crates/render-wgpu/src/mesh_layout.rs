// SPDX-License-Identifier: MIT
//! Shared explicit shader ABI for the opaque sidedness pipeline variants.
use wgpu::{BindGroupLayoutEntry as Entry, BindingType as Ty, ShaderStages as Stages};

pub(crate) fn entry(binding: u32, visibility: Stages, ty: Ty) -> Entry {
    Entry {
        binding,
        visibility,
        ty,
        count: None,
    }
}
pub(crate) fn buffer(binding: u32, visibility: Stages, uniform: bool) -> Entry {
    entry(
        binding,
        visibility,
        Ty::Buffer {
            ty: if uniform {
                wgpu::BufferBindingType::Uniform
            } else {
                wgpu::BufferBindingType::Storage { read_only: true }
            },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
    )
}
pub(crate) fn texture(binding: u32, dimension: wgpu::TextureViewDimension) -> Entry {
    entry(
        binding,
        Stages::FRAGMENT,
        Ty::Texture {
            sample_type: wgpu::TextureSampleType::Float { filterable: true },
            view_dimension: dimension,
            multisampled: false,
        },
    )
}
pub(crate) fn sampler(binding: u32) -> Entry {
    entry(
        binding,
        Stages::FRAGMENT,
        Ty::Sampler(wgpu::SamplerBindingType::Filtering),
    )
}

pub(super) fn pipeline_layout(
    device: &wgpu::Device,
    shadows: bool,
    rays: Option<&crate::ray_scene::Plan>,
) -> wgpu::PipelineLayout {
    let frame = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("mesh-frame-layout"),
        entries: &[
            buffer(0, Stages::VERTEX_FRAGMENT, true),
            buffer(1, Stages::FRAGMENT, true),
            buffer(2, Stages::VERTEX, false),
            buffer(3, Stages::FRAGMENT, true),
            buffer(4, Stages::FRAGMENT, false),
            buffer(5, Stages::FRAGMENT, true),
            texture(6, wgpu::TextureViewDimension::D2),
            sampler(7),
            texture(10, wgpu::TextureViewDimension::D3),
            sampler(11),
        ],
    });
    let mut material = vec![buffer(0, Stages::FRAGMENT, true)];
    for binding in [1, 3, 5, 7] {
        material.extend([
            texture(binding, wgpu::TextureViewDimension::D2),
            sampler(binding + 1),
        ]);
    }
    let material = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("mesh-material-layout"),
        entries: &material,
    });
    let shadow_entries = [
        buffer(0, Stages::FRAGMENT, true),
        entry(
            1,
            Stages::FRAGMENT,
            Ty::Texture {
                sample_type: wgpu::TextureSampleType::Depth,
                view_dimension: wgpu::TextureViewDimension::D2Array,
                multisampled: false,
            },
        ),
        entry(
            2,
            Stages::FRAGMENT,
            Ty::Sampler(wgpu::SamplerBindingType::Comparison),
        ),
    ];
    let shadow = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("mesh-shadow-layout"),
        entries: if shadows { &shadow_entries } else { &[] },
    });
    let ray_layout = rays.map(|plan| {
        device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("mesh-ray-layout"),
            entries: &plan.layout_entries(),
        })
    });
    let mut layouts = vec![Some(&frame), Some(&material)];
    if shadows || ray_layout.is_some() {
        layouts.push(shadows.then_some(&shadow));
    }
    if let Some(layout) = &ray_layout {
        layouts.push(Some(layout));
    }
    device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("opaque-mesh-layout"),
        bind_group_layouts: &layouts,
        immediate_size: 0,
    })
}
