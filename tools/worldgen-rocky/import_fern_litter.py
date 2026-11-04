#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Extract three pinned CC0 fern fronds into rooted, soil-projectable litter forms."""
import hashlib
import io
import json
import struct
import sys
import zlib
from pathlib import Path

import numpy as np
from PIL import Image

SOURCE_SHA256 = "10025cc15e9067b430e5be4b2628a06ac0c7f8acc49d5b2f61dddeda08a48b26"
COMPONENT_TYPES = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
TYPE_WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
OUT_OF_PLANE_SCALE = 0.35
DRY_BROWN_LINEAR = np.array([0.15, 0.105, 0.065], dtype=np.float64)
DIFFUSE_2K_SHA256 = "25168368e55d56e1d4eb97aaf72b1ea7c59b61aa75d64ebca86150a34762a583"
ALPHA_2K_SHA256 = "f51eb9b8f9f4b25a882f8fe12d1ef53ba69448bac7e71396ec1f05b684e9ea75"


def glb_chunks(raw):
    if raw[:4] != b"glTF" or struct.unpack_from("<II", raw, 4) != (2, len(raw)):
        raise ValueError("source must be one valid GLB 2.0 file")
    chunks = []
    offset = 12
    while offset < len(raw):
        size, kind = struct.unpack_from("<I4s", raw, offset)
        offset += 8
        chunks.append((kind, raw[offset : offset + size]))
        offset += size
    if offset != len(raw) or [kind for kind, _ in chunks] != [b"JSON", b"BIN\0"]:
        raise ValueError("expected exactly JSON and BIN GLB chunks")
    return json.loads(chunks[0][1]), chunks[1][1]


def accessor(doc, binary, index):
    a = doc["accessors"][index]
    if "sparse" in a:
        raise ValueError("sparse source accessors are unsupported")
    view = doc["bufferViews"][a["bufferView"]]
    dtype = COMPONENT_TYPES[a["componentType"]]
    width = TYPE_WIDTH[a["type"]]
    item_size = np.dtype(dtype).itemsize
    stride = view.get("byteStride", item_size * width)
    start = view.get("byteOffset", 0) + a.get("byteOffset", 0)
    shape = (a["count"], width) if width > 1 else (a["count"],)
    strides = (stride, item_size) if width > 1 else (stride,)
    return np.ndarray(shape, dtype, binary, start, strides=strides).copy()


def components(indices):
    parent = list(range(int(indices.max()) + 1))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[b] = a

    for tri in indices.reshape(-1, 3):
        union(int(tri[0]), int(tri[1]))
        union(int(tri[1]), int(tri[2]))
    groups = {}
    for face, tri in enumerate(indices.reshape(-1, 3)):
        groups.setdefault(find(int(tri[0])), []).append(face)
    return [(np.unique(indices.reshape(-1, 3)[faces]), np.asarray(faces)) for faces in groups.values()]


def image_bytes(doc, binary, index):
    image = doc["images"][index]
    if "uri" in image or image["mimeType"] != "image/png":
        raise ValueError("expected embedded PNG source textures")
    view = doc["bufferViews"][image["bufferView"]]
    start = view.get("byteOffset", 0)
    return binary[start : start + view["byteLength"]]


def png_chunk(kind, payload):
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)


def rgba16_png(rgb, alpha):
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("diffuse input must be an RGB8 image")
    if alpha.dtype != np.uint16 or alpha.shape != rgb.shape[:2]:
        raise ValueError("alpha input must be a matching 16-bit grayscale image")
    height, width = alpha.shape
    channels = np.empty((height, width, 4), dtype=np.uint16)
    channels[:, :, :3] = rgb.astype(np.uint16) * 257
    channels[:, :, 3] = alpha
    # PNG filter 0 keeps the conversion transparent and deterministic.
    rows = b"".join(b"\0" + row.astype(">u2", copy=False).tobytes() for row in channels)
    header = struct.pack(">IIBBBBB", width, height, 16, 6, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + png_chunk(b"IHDR", header) + png_chunk(b"IDAT", zlib.compress(rows, 9)) + png_chunk(b"IEND", b"")


def brown_albedo(diffuse_png, alpha_png):
    source = Image.open(io.BytesIO(diffuse_png)).convert("RGB")
    rgb = np.asarray(source, dtype=np.uint8).copy()
    alpha_image = Image.open(io.BytesIO(alpha_png))
    if alpha_image.mode not in ("I;16", "I;16B", "I"):
        raise ValueError(f"2K alpha source must decode as 16-bit grayscale, got {alpha_image.mode}")
    alpha = np.asarray(alpha_image, dtype=np.uint16)
    if alpha.shape != rgb.shape[:2]:
        raise ValueError("2K diffuse and alpha source dimensions differ")
    srgb = rgb.astype(np.float64) / 255.0
    linear = np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)
    luminance = linear @ np.array([0.2126, 0.7152, 0.0722])
    visible = alpha > 0
    median = float(np.median(luminance[visible]))
    shade = np.clip((luminance / max(median, 1e-6)) ** 0.72, 0.38, 1.65)
    # White-balance source chromaticity instead of erasing scan color to gray.
    chroma = linear / np.maximum(luminance[:, :, None], 1e-6)
    reference_chroma = np.median(chroma[alpha > 32767], axis=0)
    dry = shade[:, :, None] * DRY_BROWN_LINEAR[None, None, :] * (
        1.0 + 0.10 * (chroma / np.maximum(reference_chroma, 1e-6) - 1.0)
    )
    encoded = np.where(dry <= 0.0031308, 12.92 * dry, 1.055 * np.power(dry, 1.0 / 2.4) - 0.055)
    dry_rgb = np.rint(np.clip(encoded, 0.0, 1.0) * 255.0).astype(np.uint8)
    result = rgba16_png(dry_rgb, alpha)
    return result


def build_asset(source_path, output_path, diffuse_2k_path, alpha_2k_path):
    source_raw = source_path.read_bytes()
    source_hash = hashlib.sha256(source_raw).hexdigest()
    if source_hash != SOURCE_SHA256:
        raise ValueError(f"unrecognized fern_02 source GLB: {source_hash}")
    doc, binary = glb_chunks(source_raw)
    raw_images = [image_bytes(doc, binary, i) for i in range(len(doc["images"]))]
    diffuse_2k = diffuse_2k_path.read_bytes()
    alpha_2k = alpha_2k_path.read_bytes()
    diffuse_hash = hashlib.sha256(diffuse_2k).hexdigest()
    alpha_hash = hashlib.sha256(alpha_2k).hexdigest()
    if diffuse_hash != DIFFUSE_2K_SHA256 or alpha_hash != ALPHA_2K_SHA256:
        raise ValueError(f"unrecognized Poly Haven 2K maps: diffuse={diffuse_hash}, alpha={alpha_hash}")
    albedo = brown_albedo(diffuse_2k, alpha_2k)
    outputs, records = [], []
    for mesh_index in (0, 1, 2):
        primitive = doc["meshes"][mesh_index]["primitives"][0]
        attrs = primitive["attributes"]
        positions = accessor(doc, binary, attrs["POSITION"])
        normals = accessor(doc, binary, attrs["NORMAL"])
        uvs = accessor(doc, binary, attrs["TEXCOORD_0"])
        indices = accessor(doc, binary, primitive["indices"]).astype(np.uint32)
        node = doc["nodes"][mesh_index]
        if any(k in node for k in ("matrix", "rotation", "scale")):
            raise ValueError("source growth forms must have translation-only nodes")
        translation = np.asarray(node.get("translation", [0.0, 0.0, 0.0]), dtype=np.float64)
        world_positions = positions.astype(np.float64) + translation
        gallery_root = np.rint((world_positions.min(axis=0) + world_positions.max(axis=0)) * 0.5)
        candidates = []
        for vertex_ids, face_ids in components(indices):
            points = world_positions[vertex_ids]
            root_local = int(np.argmin(np.linalg.norm(points - gallery_root, axis=1)))
            root = points[root_local]
            centered = points - points.mean(axis=0)
            eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered)
            axis = eigenvectors[:, int(np.argmax(eigenvalues))]
            if np.dot(axis, points.mean(axis=0) - root) < 0.0:
                axis = -axis
            component_triangles = indices.reshape(-1, 3)[face_ids]
            triangle_positions = world_positions[component_triangles]
            geometric_area = np.linalg.norm(
                np.cross(
                    triangle_positions[:, 1] - triangle_positions[:, 0],
                    triangle_positions[:, 2] - triangle_positions[:, 0],
                ),
                axis=1,
            )
            triangle_vertex_normals = normals[component_triangles].astype(np.float64).mean(axis=1)
            triangle_vertex_normals /= np.maximum(
                np.linalg.norm(triangle_vertex_normals, axis=1, keepdims=True), 1e-12
            )
            surface_normal = (triangle_vertex_normals * geometric_area[:, None]).sum(axis=0)
            surface_normal /= np.linalg.norm(surface_normal)
            # The second PCA axis can follow arch/curl instead of the blade
            # width. Use the authored sheet normals to recover its face frame.
            axis -= surface_normal * np.dot(axis, surface_normal)
            axis /= np.linalg.norm(axis)
            projection = (points - root) @ axis
            span = float(projection.max() - projection.min())
            width_axis = np.cross(surface_normal, axis)
            width_axis /= np.linalg.norm(width_axis)
            candidates.append((span, vertex_ids, face_ids, root_local, root, axis, width_axis, surface_normal, eigenvalues, eigenvectors))
        span, vertex_ids, face_ids, root_local, root, axis, width_axis, surface_normal, eigenvalues, eigenvectors = max(candidates, key=lambda c: c[0])
        # Put the broadest transverse axis horizontally while retaining more
        # of the source pinna curl for a less planar, less rigid silhouette.
        rotation = np.stack([width_axis, np.cross(axis, width_axis), axis])
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8):
            raise ValueError("frond frame must be a proper rotation")
        local_points = (world_positions[vertex_ids] - root) @ rotation.T
        longitudinal = (world_positions[vertex_ids] - root) @ axis
        tip = float(longitudinal.max())
        if not (span > 0.1 and tip > 0.1):
            raise ValueError("selected component is too short to be a frond")
        local_points /= tip
        # Remove the living rachis arch, retaining the scanned pinna folds.
        # This is an authored resting-organ deformation, not a decay simulation.
        arch = np.polynomial.polynomial.polyfit(local_points[:, 2], local_points[:, 1], 3)
        arch_height = np.polynomial.polynomial.polyval(local_points[:, 2], arch)
        arch_slope = np.polynomial.polynomial.polyval(
            local_points[:, 2], np.polynomial.polynomial.polyder(arch)
        )
        source_height_span = float(np.ptp(local_points[:, 1]))
        local_points[:, 1] = (local_points[:, 1] - arch_height) * OUT_OF_PLANE_SCALE
        local_normals = normals[vertex_ids].astype(np.float64) @ rotation.T
        # Inverse transpose for y' = scale * (y - arch(z)).
        local_normals[:, 2] += arch_slope * local_normals[:, 1]
        local_normals[:, 1] /= OUT_OF_PLANE_SCALE
        local_normals /= np.linalg.norm(local_normals, axis=1, keepdims=True)
        remap = np.full(len(positions), -1, dtype=np.int64)
        remap[vertex_ids] = np.arange(len(vertex_ids))
        local_indices = remap[indices.reshape(-1, 3)[face_ids]].astype("<u2")
        local_positions = local_points.astype("<f4")
        local_normals = local_normals.astype("<f4")
        local_uvs = uvs[vertex_ids].astype("<f4")
        outputs.append((local_positions, local_normals, local_uvs, local_indices))
        records.append({
            "source_mesh_index": mesh_index,
            "source_mesh_name": doc["meshes"][mesh_index].get("name"),
            "source_node_name": node.get("name"),
            "source_component_vertices": int(len(vertex_ids)),
            "source_component_triangles": int(len(face_ids)),
            "source_component_root_vertex": int(vertex_ids[root_local]),
            "source_gallery_root_world": gallery_root.tolist(),
            "source_frond_root_world": root.tolist(),
            "source_long_axis": axis.tolist(),
            "source_width_axis": width_axis.tolist(),
            "source_area_weighted_sheet_normal": surface_normal.tolist(),
            "pca_second_axis_sheet_normal_alignment": float(abs(np.dot(eigenvectors[:, -2], surface_normal))),
            "width_frame": "cross(area-weighted authored vertex-normal average, projected PCA longitudinal axis)",
            "source_longitudinal_extent_m": tip,
            "output_longitudinal_extent": float(local_positions[:, 2].max() - local_positions[:, 2].min()),
            "out_of_plane_scale": OUT_OF_PLANE_SCALE,
            "removed_rachis_arch_coefficients": arch.tolist(),
            "source_normalized_height_span": source_height_span,
            "resting_normalized_height_span": float(np.ptp(local_points[:, 1])),
            "uvs_preserved_exactly": bool(np.array_equal(local_uvs, uvs[vertex_ids].astype("<f4"))),
            "normal_transform": "rotate, inverse-transpose arch removal and local-Y curl scale, normalize",
        })

    # Keep one exact alpha mask and the source normal and roughness ORM image bytes.
    images = [albedo, raw_images[0], raw_images[2]]
    binary_out = bytearray()
    views, accessors = [], []

    def append_view(payload, target=None):
        while len(binary_out) % 4:
            binary_out.append(0)
        offset = len(binary_out)
        binary_out.extend(payload)
        view = {"buffer": 0, "byteOffset": offset, "byteLength": len(payload)}
        if target is not None:
            view["target"] = target
        views.append(view)
        return len(views) - 1

    def append_accessor(array, gltf_type, component_type, target=None, bounds=False):
        view = append_view(array.tobytes(order="C"), target)
        out = {"bufferView": view, "componentType": component_type, "count": int(len(array)), "type": gltf_type}
        if bounds:
            out["min"] = array.min(axis=0).astype(float).tolist()
            out["max"] = array.max(axis=0).astype(float).tolist()
        accessors.append(out)
        return len(accessors) - 1

    meshes = []
    for index, (pos, normal, uv, indices) in enumerate(outputs):
        p = append_accessor(pos, "VEC3", 5126, 34962, True)
        n = append_accessor(normal, "VEC3", 5126, 34962)
        t = append_accessor(uv, "VEC2", 5126, 34962)
        i = append_accessor(indices.reshape(-1), "SCALAR", 5123, 34963)
        meshes.append({"name": f"fern_litter_{index + 1:02d}", "primitives": [{"attributes": {"POSITION": p, "NORMAL": n, "TEXCOORD_0": t}, "indices": i, "material": 0}]})
    image_views = [append_view(payload) for payload in images]
    gltf = {
        "asset": {"version": "2.0", "generator": "Thessa pinned fern litter converter"},
        "scene": 0,
        "scenes": [{"nodes": [0, 1, 2]}],
        "nodes": [{"mesh": i, "name": f"fern_litter_{i + 1:02d}_rooted"} for i in range(3)],
        "meshes": meshes,
        "materials": [{"name": "fern_litter_dry_cutout", "alphaMode": "MASK", "alphaCutoff": 0.5, "doubleSided": True, "normalTexture": {"index": 1, "scale": OUT_OF_PLANE_SCALE}, "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}, "metallicFactor": 0.0, "metallicRoughnessTexture": {"index": 2}}}],
        "textures": [{"sampler": 0, "source": i} for i in range(3)],
        "samplers": doc.get("samplers", [{"magFilter": 9729, "minFilter": 9987}]),
        "images": [{"bufferView": image_views[i], "mimeType": "image/png", "name": name} for i, name in enumerate(("fern_litter_dry_albedo_alpha", "fern_02_nor_gl", "fern_02_rough_orm"))],
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(binary_out)}],
    }
    json_chunk = json.dumps(gltf, separators=(",", ":"), sort_keys=True).encode("utf-8")
    json_chunk += b" " * ((-len(json_chunk)) % 4)
    binary_chunk = bytes(binary_out) + b"\0" * ((-len(binary_out)) % 4)
    total = 12 + 8 + len(json_chunk) + 8 + len(binary_chunk)
    output_path.write_bytes(b"glTF" + struct.pack("<II", 2, total) + struct.pack("<I4s", len(json_chunk), b"JSON") + json_chunk + struct.pack("<I4s", len(binary_chunk), b"BIN\0") + binary_chunk)
    out_hash = hashlib.sha256(output_path.read_bytes()).hexdigest()
    manifest = {
        "variant": "source-connected-frond-v2",
        "source": {"asset": "fern-02.glb", "sha256": source_hash, "license": "CC0-1.0", "source_page": "https://polyhaven.com/a/fern_02", "source_license": "https://polyhaven.com/license"},
        "source_texture_files": [
            {"role": "2K diffuse", "filename": diffuse_2k_path.name, "url": "https://dl.polyhaven.org/file/ph-assets/Models/png/2k/fern_02/fern_02_diff_2k.png", "sha256": diffuse_hash, "md5": "3cf4791cc57bafa48df0026ddace3b7b"},
            {"role": "2K alpha", "filename": alpha_2k_path.name, "url": "https://dl.polyhaven.org/file/ph-assets/Models/png/2k/fern_02/fern_02_alpha_2k.png", "sha256": alpha_hash, "md5": "ff18f7e8e869c58eac8f9c7d7c519f4a"},
        ],
        "conversion": "Select the longest connected vertex component from source gallery meshes 0, 1, and 2 using its PCA longitudinal extent after projecting that axis into the source-normal tangent plane. Root each at the component vertex nearest its existing rounded gallery root. The authored area-weighted sheet-normal average and projected longitudinal axis define the blade-face frame: their cross product maps true transverse width to local +X, while the mean normal maps to local +Y. This avoids mapping sheet-normal arch variance to width. Uniform scale makes positive root-to-tip extent 1.0. Subtract the least-squares cubic longitudinal rachis arch before flattening; preserve transverse pinna folds and transform normals by the full inverse-transpose shear/scale. This authors a resting fallen organ rather than projecting its living canopy arch onto the floor. Preserve 35% of source out-of-plane curl and use the same 0.35 tangent-normal relief scale as a flattened-sheet microrelief approximation (not an exact rebake of the deformed scan). Preserve source UVs; vertex normals use the inverse transpose of rotation plus curl scaling. Use pinned 2K diffuse RGB and 16-bit grayscale alpha maps; remap alpha into RGBA16 without changing samples. Linear diffuse luminance modulates authored muted dry-brown linear target [0.15, 0.105, 0.065]; 10% of median-white-balanced source chromaticity remains around that dry target, preserving subtle scan pigment/lesion variation; the full-chromaticity candidate was visually rejected for pink/purple shifts. The renderer may quantize this texture to RGBA8. This is an optical dead-frond proxy, not measured senescent tissue.",
        "textures": [
            {"role": "dry base color + cutout alpha", "source_sha256": hashlib.sha256(diffuse_2k + alpha_2k).hexdigest(), "output_sha256": hashlib.sha256(albedo).hexdigest(), "alpha_source_sha256": alpha_hash, "alpha_samples_preserved_exactly_in_rgba16": True, "source_map_resolution": [2048, 2048]},
            {"role": "normal", "source_sha256": hashlib.sha256(raw_images[0]).hexdigest(), "output_sha256": hashlib.sha256(raw_images[0]).hexdigest(), "preserved_exactly": True},
            {"role": "metallic-roughness ORM", "source_sha256": hashlib.sha256(raw_images[2]).hexdigest(), "output_sha256": hashlib.sha256(raw_images[2]).hexdigest(), "preserved_exactly": True},
        ],
        "forms": records,
        "output": {"file": output_path.name, "bytes": output_path.stat().st_size, "sha256": out_hash, "triangles": sum(int(len(o[3])) for o in outputs), "vertices": sum(len(o[0]) for o in outputs)},
        "converter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    output_path.with_name("fern-litter-01-conversion.json").write_text(json.dumps(manifest, indent=2) + "\n")
    output_path.with_name("fern-litter-01.LICENSE.txt").write_text(
        "Fern litter derivative from Poly Haven fern_02 (https://polyhaven.com/a/fern_02), CC0-1.0 (https://polyhaven.com/license).\n"
        "The extracted connected frond components retain source UVs, normal map, and roughness map. A PCA longitudinal axis is projected into the source-normal tangent plane; the area-weighted source sheet normal and that axis define the blade-face frame, so true transverse width stays in the soil plane while curl follows local Y. Geometry is rooted, uniformly normalized for native soil projection, and retains 35% of source out-of-plane pinna curl. Base-color RGB is an authored muted dry-brown remap from source linear luminance. The embedded base-color alpha uses the pinned Poly Haven 2K 16-bit alpha map, copied sample-for-sample into RGBA16; rendering may quantize it to RGBA8. The result is an optical dead-frond proxy, not measured dead tissue.\n"
        "The source asset is public domain under CC0 1.0; no attribution is required.\n"
    )
    print(json.dumps(manifest["output"], indent=2))


if __name__ == "__main__":
    if len(sys.argv) != 5:
        raise SystemExit(f"usage: {Path(sys.argv[0]).name} SOURCE_FERN_GLB OUTPUT_GLB DIFFUSE_2K_PNG ALPHA_2K_PNG")
    build_asset(*(Path(arg) for arg in sys.argv[1:]))
