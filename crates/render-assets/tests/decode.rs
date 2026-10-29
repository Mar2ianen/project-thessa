use serde_json::{Value, json};
use thessa_render_assets::{Limits, decode_glb};
fn fixture() -> (Value, Vec<u8>) {
    let mut binary = Vec::new();
    for v in [
        -1.0_f32, -1.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 1.0, 0.0, 0.5, 1.0,
    ] {
        binary.extend(v.to_le_bytes());
    }
    for i in 0..3_u32 {
        binary.extend(i.to_le_bytes());
    }
    (
        json!({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"translation":[2,3,4],"children":[1]},{"mesh":0,"scale":[-2,3,4]}],
        "meshes":[{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2},"indices":3}]}],"buffers":[{"byteLength":binary.len()}],
        "bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":36},{"buffer":0,"byteOffset":72,"byteLength":24},{"buffer":0,"byteOffset":96,"byteLength":12}],
        "accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[-1,-1,0],"max":[1,1,0]},
            {"bufferView":1,"componentType":5126,"count":3,"type":"VEC3"},{"bufferView":2,"componentType":5126,"count":3,"type":"VEC2"},{"bufferView":3,"componentType":5125,"count":3,"type":"SCALAR"}]}),
        binary,
    )
}
fn encode(doc: &Value, blob: &[u8]) -> Vec<u8> {
    let mut text = serde_json::to_vec(doc).unwrap();
    while !text.len().is_multiple_of(4) {
        text.push(b' ');
    }
    let mut blob = blob.to_vec();
    while !blob.len().is_multiple_of(4) {
        blob.push(0);
    }
    let mut out = Vec::new();
    for word in [
        0x46546c67,
        2,
        (12 + 8 + text.len() + 8 + blob.len()) as u32,
        text.len() as u32,
        0x4e4f534a,
    ] {
        out.extend(word.to_le_bytes());
    }
    out.extend(text);
    out.extend((blob.len() as u32).to_le_bytes());
    out.extend(0x004e4942u32.to_le_bytes());
    out.extend(blob);
    out
}
#[test]
fn nested_nonuniform_mirrored_transform_preserves_normals_and_winding() {
    let (doc, blob) = fixture();
    let source = decode_glb(&encode(&doc, &blob), Limits::default()).unwrap();
    assert_eq!(source.primitives[0].vertices[0].position, [4.0, 0.0, 4.0]);
    assert_eq!(source.primitives[0].vertices[0].normal, [0.0, 0.0, 1.0]);
    assert_eq!(source.primitives[0].indices, [0, 2, 1]);
    assert_eq!(source.materials.len(), 1);
}

#[test]
fn cutout_import_is_explicit_and_preserves_coverage_instead_of_accepting_blend() {
    let (mut doc, blob) = fixture();
    doc["meshes"][0]["primitives"][0]["material"] = json!(0);
    for cutoff in [None, Some(0.0), Some(0.37), Some(1.5)] {
        doc["materials"] = json!([{"alphaMode":"MASK","doubleSided":true,
            "pbrMetallicRoughness":{"baseColorFactor":[0.2,0.4,0.6,0.75]}}]);
        if let Some(cutoff) = cutoff {
            doc["materials"][0]["alphaCutoff"] = json!(cutoff);
        }
        let bytes = encode(&doc, &blob);
        assert!(decode_glb(&bytes, Limits::default()).is_err());
        let source = thessa_render_assets::decode_glb_cutouts(&bytes, Limits::default()).unwrap();
        assert_eq!(source.materials[0].alpha_cutoff, cutoff.unwrap_or(0.5));
        assert_eq!(
            source.materials[0].alpha_mode,
            thessa_render_core::mesh::AlphaMode::Mask
        );
        assert_eq!(source.materials[0].base_color, [0.2, 0.4, 0.6, 0.75]);
        assert!(source.materials[0].double_sided);
    }
    doc["materials"][0]["alphaMode"] = json!("OPAQUE");
    assert_eq!(
        thessa_render_assets::decode_glb_cutouts(&encode(&doc, &blob), Limits::default())
            .unwrap()
            .materials[0]
            .alpha_mode,
        thessa_render_core::mesh::AlphaMode::Opaque
    );
    doc["materials"][0]["alphaMode"] = json!("BLEND");
    assert!(
        thessa_render_assets::decode_glb_cutouts(&encode(&doc, &blob), Limits::default()).is_err()
    );
    doc["materials"][0]["alphaMode"] = json!("MASK");
    doc["extensionsUsed"] = json!(["KHR_materials_unlit"]);
    assert!(
        thessa_render_assets::decode_glb_cutouts(&encode(&doc, &blob), Limits::default()).is_err()
    );
}

#[test]
fn explicit_rest_pose_preserves_static_transforms_and_does_not_relax_other_limits() {
    let (doc, blob) = fixture();
    let bytes = encode(&doc, &blob);
    let strict = decode_glb(&bytes, Limits::default()).unwrap();
    let rest = thessa_render_assets::decode_glb_rest_pose(&bytes, Limits::default()).unwrap();
    assert_eq!(strict.primitives[0].vertices, rest.primitives[0].vertices);
    assert_eq!(strict.primitives[0].indices, rest.primitives[0].indices);
    assert!(
        thessa_render_assets::decode_glb_rest_pose(
            &bytes,
            Limits {
                nodes: 1,
                ..Default::default()
            }
        )
        .is_err()
    );
    let mut external = doc;
    external["buffers"][0]["uri"] = json!("file:///not-an-asset");
    assert!(
        thessa_render_assets::decode_glb_rest_pose(&encode(&external, &blob), Limits::default())
            .is_err()
    );
}

#[test]
fn emissive_strength_and_blend_are_opt_in_and_keep_hdr_linear_values() {
    let (mut doc, blob) = fixture();
    doc["meshes"][0]["primitives"][0]["material"] = json!(0);
    doc["extensionsUsed"] = json!(["KHR_materials_emissive_strength"]);
    doc["materials"] = json!([{"alphaMode":"BLEND","emissiveFactor":[0.1,0.5,0.9],
        "pbrMetallicRoughness":{"baseColorFactor":[0.,0.,0.,0.25]},
        "extensions":{"KHR_materials_emissive_strength":{"emissiveStrength":4.}}}]);
    let bytes = encode(&doc, &blob);
    assert!(decode_glb(&bytes, Limits::default()).is_err());
    assert!(thessa_render_assets::decode_glb_cutouts(&bytes, Limits::default()).is_err());
    let source = thessa_render_assets::decode_glb_surfaces(&bytes, Limits::default()).unwrap();
    assert_eq!(
        source.materials[0].alpha_mode,
        thessa_render_core::mesh::AlphaMode::Blend
    );
    assert_eq!(source.materials[0].base_color[3], 0.25);
    assert_eq!(source.materials[0].emission_rgb, [0.4, 2., 3.6]);
    doc["materials"][0]["extensions"]["KHR_materials_emissive_strength"]["emissiveStrength"] =
        json!(-1.);
    assert!(
        thessa_render_assets::decode_glb_surfaces(&encode(&doc, &blob), Limits::default()).is_err()
    );
    doc["extensionsUsed"] = json!(["KHR_materials_transmission"]);
    assert!(
        thessa_render_assets::decode_glb_surfaces(&encode(&doc, &blob), Limits::default()).is_err()
    );
}
#[test]
fn embedded_image_color_bytes_and_sampler_are_preserved_and_budgeted() {
    let (mut doc, mut blob) = fixture();
    let mut png = std::io::Cursor::new(Vec::new());
    image::RgbaImage::from_raw(1, 1, vec![128, 64, 32, 255])
        .unwrap()
        .write_to(&mut png, image::ImageFormat::Png)
        .unwrap();
    doc["bufferViews"]
        .as_array_mut()
        .unwrap()
        .push(json!({"buffer":0,"byteOffset":blob.len(),"byteLength":png.get_ref().len()}));
    blob.extend(png.into_inner());
    doc["buffers"][0]["byteLength"] = json!(blob.len());
    doc["images"] = json!([{"bufferView":4,"mimeType":"image/png"}]);
    doc["textures"] = json!([{"source":0,"sampler":0}]);
    doc["samplers"] = json!([{"wrapS":33071,"wrapT":33648,"magFilter":9728,"minFilter":9729}]);
    doc["materials"] = json!([{"pbrMetallicRoughness":{"baseColorTexture":{"index":0}}}]);
    doc["meshes"][0]["primitives"][0]["material"] = json!(0);
    let data = encode(&doc, &blob);
    let source = decode_glb(&data, Limits::default()).unwrap();
    assert_eq!(source.images[0].rgba8, [128, 64, 32, 255]);
    assert!(!source.textures[0].linear);
    assert!(source.textures[0].min_linear);
    assert!(
        decode_glb(
            &data,
            Limits {
                decoded_image_bytes: 3,
                ..Default::default()
            }
        )
        .is_err()
    );
    doc["images"][0] = json!({"uri":"file:///etc/passwd"});
    assert!(
        decode_glb(&encode(&doc, &blob), Limits::default())
            .unwrap_err()
            .contains("external")
    );
}
#[test]
fn invalid_ranges_indices_modes_and_budgets_fail_before_publication() {
    let (doc, blob) = fixture();
    let bytes = encode(&doc, &blob);
    for limits in [
        Limits {
            file_bytes: 1,
            ..Default::default()
        },
        Limits {
            vertices: 2,
            ..Default::default()
        },
        Limits {
            indices: 2,
            ..Default::default()
        },
        Limits {
            nodes: 1,
            ..Default::default()
        },
    ] {
        assert!(decode_glb(&bytes, limits).is_err());
    }
    let mut bad = doc.clone();
    bad["buffers"][0]["byteLength"] = json!(99999);
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
    let mut bad = doc.clone();
    bad["accessors"][0]["count"] = json!(usize::MAX);
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
    let mut bad = doc.clone();
    bad["accessors"][0]["byteOffset"] = json!(32);
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
    let mut bad = doc.clone();
    bad["accessors"][1]["count"] = json!(2);
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
    let mut bad = doc.clone();
    bad["accessors"][3]["componentType"] = json!(5126);
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
    let mut bad = doc.clone();
    bad["accessors"][2]["componentType"] = json!(5125);
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
    let mut bad = doc.clone();
    bad["nodes"][0] = json!({"children":[1],"matrix":[1,0,0,0.25,0,1,0,0,0,0,1,0,0,0,0,1]});
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
    let mut bad = doc.clone();
    bad["meshes"][0]["primitives"][0]["mode"] = json!(1);
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
    let mut binary = blob.clone();
    binary[96..100].copy_from_slice(&99_u32.to_le_bytes());
    assert!(decode_glb(&encode(&doc, &binary), Limits::default()).is_err());
    let mut bad = doc;
    bad["nodes"][1]["scale"] = json!([0, 1, 1]);
    assert!(decode_glb(&encode(&bad, &blob), Limits::default()).is_err());
}
#[test]
fn shipped_x15_has_exact_geometry_material_roles_and_authored_extent() {
    let source = decode_glb(
        include_bytes!("../../../assets/models/north_american_x-15_plane.glb"),
        Default::default(),
    )
    .unwrap();
    assert_eq!(source.primitives.len(), 2);
    assert_eq!(source.images.len(), 6);
    assert_eq!(
        source
            .primitives
            .iter()
            .map(|p| p.vertices.len())
            .sum::<usize>(),
        86214
    );
    assert_eq!(
        source
            .primitives
            .iter()
            .map(|p| p.indices.len() / 3)
            .sum::<usize>(),
        78465
    );
    assert_eq!(
        source.images.iter().map(|i| i.rgba8.len()).sum::<usize>(),
        100663296
    );
    for p in &source.primitives {
        let m = &source.materials[p.material];
        assert!(m.double_sided);
        assert!(
            m.base_texture.is_some()
                && m.normal_texture.is_some()
                && m.metallic_roughness_texture == m.occlusion_texture
        );
    }
    let xs: Vec<_> = source
        .primitives
        .iter()
        .flat_map(|p| p.vertices.iter().map(|v| v.position[0]))
        .collect();
    let extent = xs.iter().copied().fold(f32::NEG_INFINITY, f32::max)
        - xs.iter().copied().fold(f32::INFINITY, f32::min);
    assert!(
        (extent - 17.794085).abs() < 0.00002,
        "source length {extent}"
    );
}
