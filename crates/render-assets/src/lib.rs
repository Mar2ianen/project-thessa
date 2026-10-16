//! Bounded static embedded GLB subset. Scene transforms are flattened once;
//! images/materials remain neutral CPU data. Unsupported semantics fail loudly.
#![forbid(unsafe_code)]
use glam::{DMat3, DMat4, DVec3};
use std::io::Cursor;
use thessa_render_core::mesh::*;

#[derive(Clone, Copy)]
pub struct Limits {
    pub file_bytes: usize,
    pub vertices: usize,
    pub indices: usize,
    pub texture_edge: u32,
    pub decoded_image_bytes: u64,
    pub nodes: usize,
}
impl Default for Limits {
    fn default() -> Self {
        Self {
            file_bytes: 64 << 20,
            vertices: 2_000_000,
            indices: 6_000_000,
            texture_edge: 4096,
            decoded_image_bytes: 128 << 20,
            nodes: 4096,
        }
    }
}

pub fn decode_glb(bytes: &[u8], limits: Limits) -> Result<MeshAsset, String> {
    if bytes.len() > limits.file_bytes {
        return Err("GLB file budget exceeded".into());
    }
    let glb = gltf::Gltf::from_slice(bytes).map_err(|e| e.to_string())?;
    let blob = glb.blob.as_deref().ok_or("embedded GLB binary required")?;
    if glb.buffers().any(|b| b.length() > blob.len())
        || glb.views().any(|v| {
            v.offset()
                .checked_add(v.length())
                .is_none_or(|end| end > blob.len() || end > v.buffer().length())
        })
    {
        return Err("GLB buffer/view outside embedded binary".into());
    }
    if glb
        .buffers()
        .any(|b| !matches!(b.source(), gltf::buffer::Source::Bin))
        || glb
            .images()
            .any(|i| !matches!(i.source(), gltf::image::Source::View { .. }))
    {
        return Err("external GLB buffers/images are unsupported".into());
    }
    // gltf's utility readers assume valid byte ranges and sparse counts.
    // Keep this subset dense and validate checked arithmetic before creating
    // any iterator (including unused accessors, to avoid ambiguous assets).
    for accessor in glb.accessors() {
        if accessor.sparse().is_some() || accessor.count() == 0 {
            return Err("sparse/empty GLB accessors unsupported".into());
        }
        let view = accessor.view().ok_or("dense GLB buffer view required")?;
        let stride = view.stride().unwrap_or(accessor.size());
        let end = accessor
            .count()
            .checked_sub(1)
            .and_then(|n| n.checked_mul(stride))
            .and_then(|n| n.checked_add(accessor.size()))
            .and_then(|n| n.checked_add(accessor.offset()));
        if stride < accessor.size() || end.is_none_or(|end| end > view.length()) {
            return Err("GLB accessor outside buffer view".into());
        }
    }
    if glb.animations().len() != 0
        || glb.skins().len() != 0
        || glb.nodes().len() > limits.nodes
        || glb.extensions_used().next().is_some()
        || glb.extensions_required().next().is_some()
    {
        return Err("GLB animations/skins/extensions or node budget unsupported".into());
    }
    let mut images = Vec::new();
    let mut remaining = limits.decoded_image_bytes;
    for image in glb.images() {
        let gltf::image::Source::View { view, .. } = image.source() else {
            unreachable!()
        };
        let encoded = blob
            .get(
                view.offset()
                    ..view
                        .offset()
                        .checked_add(view.length())
                        .ok_or("image range overflow")?,
            )
            .ok_or("image range outside GLB binary")?;
        let mut reader = image::ImageReader::new(Cursor::new(encoded))
            .with_guessed_format()
            .map_err(|e| e.to_string())?;
        if !matches!(
            reader.format(),
            Some(image::ImageFormat::Png | image::ImageFormat::Jpeg)
        ) {
            return Err("only embedded PNG/JPEG supported".into());
        }
        let mut image_limits = image::Limits::default();
        image_limits.max_image_width = Some(limits.texture_edge);
        image_limits.max_image_height = Some(limits.texture_edge);
        image_limits.max_alloc = Some(remaining);
        reader.limits(image_limits);
        let (w, h) = image::ImageReader::new(Cursor::new(encoded))
            .with_guessed_format()
            .map_err(|e| e.to_string())?
            .into_dimensions()
            .map_err(|e| e.to_string())?;
        if w > limits.texture_edge
            || h > limits.texture_edge
            || u64::from(w)
                .checked_mul(u64::from(h))
                .and_then(|n| n.checked_mul(4))
                .is_none_or(|n| n > remaining)
        {
            return Err("decoded image budget exceeded".into());
        }
        let decoded = reader.decode().map_err(|e| e.to_string())?.to_rgba8();
        remaining = remaining
            .checked_sub(decoded.as_raw().len() as u64)
            .ok_or("decoded image budget exceeded")?;
        images.push(Image {
            width: decoded.width(),
            height: decoded.height(),
            rgba8: decoded.into_raw(),
        });
    }
    let wrap = |w| match w {
        gltf::texture::WrappingMode::ClampToEdge => Wrap::Clamp,
        gltf::texture::WrappingMode::MirroredRepeat => Wrap::Mirror,
        _ => Wrap::Repeat,
    };
    let textures = glb
        .textures()
        .map(|t| {
            let s = t.sampler();
            Texture {
                image: t.source().index(),
                wrap: [wrap(s.wrap_s()), wrap(s.wrap_t())],
                linear: !matches!(s.mag_filter(), Some(gltf::texture::MagFilter::Nearest)),
                min_linear: !matches!(
                    s.min_filter(),
                    Some(
                        gltf::texture::MinFilter::Nearest
                            | gltf::texture::MinFilter::NearestMipmapNearest
                            | gltf::texture::MinFilter::NearestMipmapLinear
                    )
                ),
            }
            // minification metadata is preserved even though this subset has
            // one source mip; renderer-generated mip chains are a later layer.
        })
        .collect();
    let mut materials = Vec::new();
    for m in glb.materials() {
        if m.alpha_mode() != gltf::material::AlphaMode::Opaque
            || m.emissive_factor() != [0.0; 3]
            || m.emissive_texture().is_some()
        {
            return Err("only opaque non-emissive materials supported".into());
        }
        let pbr = m.pbr_metallic_roughness();
        let texture = |t: Option<gltf::texture::Info<'_>>| -> Result<Option<usize>, String> {
            t.map(|t| {
                if t.tex_coord() != 0 {
                    Err("only TEXCOORD_0 supported".into())
                } else {
                    Ok(t.texture().index())
                }
            })
            .transpose()
        };
        if m.normal_texture().is_some_and(|t| t.tex_coord() != 0)
            || m.occlusion_texture().is_some_and(|t| t.tex_coord() != 0)
        {
            return Err("only TEXCOORD_0 supported".into());
        }
        materials.push(Material {
            base_color: pbr.base_color_factor(),
            base_texture: texture(pbr.base_color_texture())?,
            normal_texture: m.normal_texture().map(|t| t.texture().index()),
            normal_scale: m.normal_texture().map_or(1.0, |t| t.scale()),
            metallic_roughness_texture: texture(pbr.metallic_roughness_texture())?,
            metallic: pbr.metallic_factor(),
            roughness: pbr.roughness_factor(),
            occlusion_texture: m.occlusion_texture().map(|t| t.texture().index()),
            occlusion_strength: m.occlusion_texture().map_or(1.0, |t| t.strength()),
            double_sided: m.double_sided(),
        });
    }
    materials.push(Material::default());
    let scene = glb
        .default_scene()
        .or_else(|| glb.scenes().next())
        .ok_or("GLB has no scene")?;
    let mut stack: Vec<_> = scene.nodes().map(|n| (n, DMat4::IDENTITY)).collect();
    let mut visited = std::collections::BTreeSet::new();
    let mut primitives = Vec::new();
    let (mut vertex_count, mut index_count) = (0usize, 0usize);
    while let Some((node, parent)) = stack.pop() {
        if !visited.insert(node.index()) {
            return Err("GLB cycle/shared node unsupported".into());
        }
        let local = node.transform().matrix().map(|c| c.map(f64::from));
        let transform = parent * DMat4::from_cols_array_2d(&local);
        if !transform.is_finite()
            || !transform.determinant().is_finite()
            || transform.determinant().abs() < 1e-20
            || (transform.row(3) - glam::DVec4::W).abs().max_element() > 1e-12
        {
            return Err("invalid GLB node transform".into());
        }
        let normal_transform = DMat3::from_mat4(transform).inverse().transpose();
        stack.extend(node.children().map(|n| (n, transform)));
        if let Some(mesh) = node.mesh() {
            for p in mesh.primitives() {
                if p.mode() != gltf::mesh::Mode::Triangles
                    || p.morph_targets().next().is_some()
                    || p.attributes().any(|(semantic, _)| {
                        !matches!(
                            semantic,
                            gltf::Semantic::Positions
                                | gltf::Semantic::Normals
                                | gltf::Semantic::TexCoords(0)
                        )
                    })
                {
                    return Err(
                        "only static triangles with POSITION/NORMAL/TEXCOORD_0 supported".into(),
                    );
                }
                for (semantic, a) in p.attributes() {
                    use gltf::accessor::{DataType as T, Dimensions as D};
                    let valid = match semantic {
                        gltf::Semantic::Positions | gltf::Semantic::Normals => {
                            a.data_type() == T::F32 && a.dimensions() == D::Vec3 && !a.normalized()
                        }
                        gltf::Semantic::TexCoords(0) => {
                            a.dimensions() == D::Vec2
                                && match a.data_type() {
                                    T::F32 => !a.normalized(),
                                    T::U8 | T::U16 => a.normalized(),
                                    _ => false,
                                }
                        }
                        _ => false,
                    };
                    if !valid {
                        return Err("invalid mesh attribute component/dimension type".into());
                    }
                }
                if p.indices().is_some_and(|a| {
                    !matches!(
                        a.data_type(),
                        gltf::accessor::DataType::U8
                            | gltf::accessor::DataType::U16
                            | gltf::accessor::DataType::U32
                    ) || a.dimensions() != gltf::accessor::Dimensions::Scalar
                        || a.normalized()
                }) {
                    return Err("invalid mesh index component/dimension type".into());
                }
                let reader = p.reader(|_| Some(blob));
                let positions = reader.read_positions().ok_or("mesh POSITION required")?;
                vertex_count = vertex_count
                    .checked_add(positions.len())
                    .ok_or("vertex count overflow")?;
                if vertex_count > limits.vertices {
                    return Err("mesh vertex budget exceeded".into());
                }
                if positions.len() > u32::MAX as usize
                    || p.get(&gltf::Semantic::Normals)
                        .is_some_and(|a| a.count() != positions.len())
                    || p.get(&gltf::Semantic::TexCoords(0))
                        .is_some_and(|a| a.count() != positions.len())
                {
                    return Err("mesh attribute count mismatch".into());
                }
                let requested_indices = p.indices().map_or(positions.len(), |a| a.count());
                if index_count
                    .checked_add(requested_indices)
                    .is_none_or(|n| n > limits.indices)
                {
                    return Err("mesh index budget exceeded".into());
                }
                let material = &materials[p.material().index().unwrap_or(materials.len() - 1)];
                if [
                    material.base_texture,
                    material.normal_texture,
                    material.metallic_roughness_texture,
                    material.occlusion_texture,
                ]
                .iter()
                .any(Option::is_some)
                    && p.get(&gltf::Semantic::TexCoords(0)).is_none()
                {
                    return Err("textured primitive requires TEXCOORD_0".into());
                }
                let normals: Vec<_> = reader
                    .read_normals()
                    .ok_or("mesh NORMAL required")?
                    .collect();
                let uv: Vec<_> = reader
                    .read_tex_coords(0)
                    .map(|v| v.into_f32().collect())
                    .unwrap_or_else(|| vec![[0.0; 2]; positions.len()]);
                if normals.len() != positions.len() || uv.len() != positions.len() {
                    return Err("mesh attribute count mismatch".into());
                }
                let vertices = positions
                    .zip(normals)
                    .zip(uv)
                    .map(|((p, n), uv)| Vertex {
                        position: transform
                            .transform_point3(DVec3::from(p.map(f64::from)))
                            .as_vec3()
                            .to_array(),
                        normal: (normal_transform * DVec3::from(n.map(f64::from)))
                            .normalize()
                            .as_vec3()
                            .to_array(),
                        uv,
                    })
                    .collect::<Vec<_>>();
                let mut indices: Vec<u32> = reader
                    .read_indices()
                    .map(|i| i.into_u32().collect())
                    .unwrap_or_else(|| (0..vertices.len() as u32).collect());
                index_count = index_count
                    .checked_add(indices.len())
                    .ok_or("index count overflow")?;
                if index_count > limits.indices {
                    return Err("mesh index budget exceeded".into());
                }
                if transform.determinant() < 0.0 {
                    for t in indices.as_chunks_mut::<3>().0 {
                        t.swap(1, 2);
                    }
                }
                primitives.push(Primitive {
                    vertices,
                    indices,
                    material: p.material().index().unwrap_or(materials.len() - 1),
                });
            }
        }
    }
    let asset = MeshAsset {
        primitives,
        materials,
        textures,
        images,
    };
    asset.validate().map_err(str::to_owned)?;
    Ok(asset)
}
