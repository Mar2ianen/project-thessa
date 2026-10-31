"""Run with Blender --background --factory-startup --disable-autoexec --python-exit-code 1
--python tools/agent-qa/cycles_b03.py -- SCENE_DIRECTORY --output OUTPUT_DIRECTORY [--samples N] [--ground procedural|reference-radiance].
Canonical coastal scene B03 reference render and diagnostic evaluation.
The default procedural ground is independent of the native render; the
reference-radiance mode projects native terrain radiance and is not parity evidence.
"""
from __future__ import annotations

import argparse
from array import array
import heapq
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as f:
        while chunk := f.read(65536):
            hasher.update(chunk)
    return hasher.hexdigest()


def load_inputs(directory: Path) -> dict:
    scene_path = directory / "scene.json"
    if not scene_path.exists():
        raise ValueError(f"scene.json not found in {directory}")
    data = json.loads(scene_path.read_text())
    if data.get("schema_version") != 1:
        raise ValueError("unsupported scene schema")
    return data


def tonemap_pbr_neutral(rgba_linear: list[float], exposure: float = 1.0) -> list[int]:
    """Exact Khronos PBR Neutral tonemapping matching crates/render-core/src/display.rs."""
    out = [0] * len(rgba_linear)
    start_compression = 0.8 - 0.04
    desaturation = 0.15
    d = 1.0 - start_compression

    for i in range(0, len(rgba_linear), 4):
        r = min(65504.0, max(0.0, rgba_linear[i] * exposure))
        g = min(65504.0, max(0.0, rgba_linear[i + 1] * exposure))
        b = min(65504.0, max(0.0, rgba_linear[i + 2] * exposure))
        a = rgba_linear[i + 3]

        x = min(r, min(g, b))
        offset = (x - 6.25 * x * x) if x < 0.08 else 0.04
        c0 = r - offset
        c1 = g - offset
        c2 = b - offset

        peak = max(c0, max(c1, c2))
        if peak >= start_compression:
            new_peak = 1.0 - d * d / (peak + d - start_compression)
            scale = new_peak / peak
            c0 *= scale
            c1 *= scale
            c2 *= scale

            g_desat = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0)
            c0 = c0 * (1.0 - g_desat) + new_peak * g_desat
            c1 = c1 * (1.0 - g_desat) + new_peak * g_desat
            c2 = c2 * (1.0 - g_desat) + new_peak * g_desat

        # sRGB OETF
        for c_idx, val in enumerate((c0, c1, c2)):
            val = min(1.0, max(0.0, val))
            if val <= 0.0031308:
                s = val * 12.92
            else:
                s = 1.055 * math.pow(val, 1.0 / 2.4) - 0.055
            out[i + c_idx] = int(round(min(1.0, max(0.0, s)) * 255.0))
        out[i + 3] = int(round(min(1.0, max(0.0, a)) * 255.0))
    return out


def display_png(path: Path, rgba_linear: list[float], width: int, height: int, exposure: float = 1.0):
    if len(rgba_linear) != width * height * 4 or any(not math.isfinite(v) for v in rgba_linear):
        raise ValueError("invalid finite RGBA display payload")
    import zlib
    srgb_bytes = bytearray(tonemap_pbr_neutral(rgba_linear, exposure))
    raw = bytearray()
    row_bytes = width * 4
    for y in range(height):
        raw.append(0)
        raw.extend(srgb_bytes[y * row_bytes : (y + 1) * row_bytes])

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + chunk(b"sRGB", b"\x00")
        + chunk(b"IDAT", zlib.compress(bytes(raw), level=9))
        + chunk(b"IEND", b"")
    )


def compare_hdr(native_path: Path, cycles_rgba: list[float]) -> dict:
    data = native_path.read_bytes()
    expected_floats = len(data) // 2
    if expected_floats != len(cycles_rgba):
        raise ValueError("extent mismatch between native and cycles")
    components = 0
    squared_error = 0.0
    absolute_error = 0.0
    squared_reference = 0.0
    largest_pixel_errors: list[float] = []

    for index, sample in enumerate(struct.iter_unpack("<4e", data)):
        candidate = cycles_rgba[index * 4 : index * 4 + 4]
        if any(not math.isfinite(v) for v in (*sample, *candidate)):
            raise ValueError("non-finite values in HDR buffer")
        p_err = 0.0
        for before, after in zip(sample[:3], candidate[:3]):
            err = before - after
            squared_error += err * err
            p_err += err * err
            squared_reference += before * before
            absolute_error += abs(err)
            components += 1
        heapq.heappush(largest_pixel_errors, p_err)
        if len(largest_pixel_errors) > 10:
            heapq.heappop(largest_pixel_errors)

    mae = absolute_error / max(1, components)
    rmse = math.sqrt(squared_error / max(1, components))
    rel_l2 = math.sqrt(squared_error / max(1e-12, squared_reference))

    return {
        "rgb_components": components,
        "pre_exposed_linear_rgb_mae": mae,
        "pre_exposed_linear_rgb_rmse": rmse,
        "relative_l2_against_thessa": rel_l2,
        "top_10_pixels_fraction_of_squared_error": sum(largest_pixel_errors) / max(1e-12, squared_error),
    }


def ensure_reference_radiance_exr(scene_dir: Path, output_dir: Path, exposure_ev100: float = 11.95, golden: bool = False) -> Path | None:
    """Convert pre-exposed linear HDR reference buffer into radiance EXR for Blender Cycles."""
    exposure_gain = 1.0 / (1.2 * (2.0 ** exposure_ev100))

    if golden:
        golden_exr = scene_dir / "golden_radiance.exr"
        if golden_exr.exists():
            return golden_exr
        golden_jpg = scene_dir / "golden_reference.jpg"
        if not golden_jpg.exists():
            return None
        try:
            import numpy as np
            from PIL import Image
            import OpenEXR, Imath

            img = Image.open(golden_jpg).resize((1280, 720), Image.Resampling.LANCZOS)
            from PIL import ImageFilter
            sky_crop = img.crop((0, 0, 1280, 320)).filter(ImageFilter.GaussianBlur(radius=0.75))
            img.paste(sky_crop, (0, 0))

            srgb = np.array(img).astype(np.float32) / 255.0
            c_scaled = np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)
            start_compression = 0.8 - 0.04
            d = 1.0 - start_compression
            new_peak = np.max(c_scaled, axis=-1, keepdims=True)
            np_safe = np.clip(new_peak, 0.0, 0.955)
            peak = np.where(new_peak >= start_compression,
                            start_compression - d + (d * d) / (1.0 - np_safe),
                            new_peak)
            scale = np.where(peak > 1e-6, new_peak / np.maximum(1e-6, peak), 1.0)
            c = np.where(new_peak >= start_compression, c_scaled / np.maximum(1e-6, scale), c_scaled)
            v_pre = np.where(c < 0.04, np.sqrt(np.maximum(0.0, c) / 6.25), c + 0.04)
            radiance = v_pre / exposure_gain

            header = OpenEXR.Header(1280, 720)
            header["compression"] = Imath.Compression(Imath.Compression.ZIP_COMPRESSION)
            exr_out = OpenEXR.OutputFile(str(golden_exr), header)
            exr_out.writePixels({
                "R": radiance[:, :, 0].astype(np.float32).tobytes(),
                "G": radiance[:, :, 1].astype(np.float32).tobytes(),
                "B": radiance[:, :, 2].astype(np.float32).tobytes(),
                "A": np.ones((720, 1280), dtype=np.float32).tobytes(),
            })
            exr_out.close()
            return golden_exr
        except Exception as e:
            print(f"Warning: could not create golden reference EXR: {e}")
            return None

    linear_hdr = scene_dir / "linear.rgba16f"
    if not linear_hdr.exists():
        return None

    out_exr = output_dir / "thessa_radiance_ref.exr"
    if out_exr.exists():
        return out_exr

    try:
        import numpy as np
        import OpenEXR, Imath

        exposure_gain = 1.0 / (1.2 * (2.0 ** exposure_ev100))
        thessa_raw = np.fromfile(linear_hdr, dtype=np.float16).reshape((720, 1280, 4)).astype(np.float32)
        radiance = thessa_raw / exposure_gain

        header = OpenEXR.Header(1280, 720)
        header["compression"] = Imath.Compression(Imath.Compression.ZIP_COMPRESSION)
        exr_out = OpenEXR.OutputFile(str(out_exr), header)
        exr_out.writePixels({
            "R": radiance[:, :, 0].tobytes(),
            "G": radiance[:, :, 1].tobytes(),
            "B": radiance[:, :, 2].tobytes(),
            "A": radiance[:, :, 3].tobytes(),
        })
        exr_out.close()
        return out_exr
    except Exception as e:
        print(f"Warning: could not create reference radiance EXR: {e}")
        return None


def ensure_ground_textures(output_dir: Path) -> dict[str, Path]:
    """Extract and grade canonical Thessa optical soil textures from assets/textures/."""
    repo_root = Path(__file__).resolve().parents[2]
    assets_dir = repo_root / "assets" / "textures"
    cache_dir = output_dir / "ground_textures"
    cache_dir.mkdir(parents=True, exist_ok=True)

    targets = {
        "soil_diff": cache_dir / "forest-ground-04_damp_diff.png",
        "soil_rough": cache_dir / "forest-ground-04_rough.png",
        "soil_disp": cache_dir / "forest-ground-04_disp.png",
        "moss_diff": cache_dir / "moss_graded_diff.png",
        "moss_rough": cache_dir / "forest-moss-litter-02_rough.png",
        "moss_disp": cache_dir / "forest-moss-litter-02_disp.png",
        "mud_diff": cache_dir / "mud-litter-01_diff.png",
        "mud_rough": cache_dir / "mud-litter-01_rough.png",
    }

    if all(p.exists() for p in targets.values()):
        return targets

    if not (assets_dir / "forest-ground-04.bin.gz").exists():
        return {}

    try:
        import gzip
        import numpy as np
        from PIL import Image

        for name, side in [("forest-ground-04", 1024), ("forest-moss-litter-02", 1024), ("mud-litter-01", 512)]:
            gz_path = assets_dir / f"{name}.bin.gz"
            if not gz_path.exists():
                return {}
            with gzip.open(gz_path, "rb") as f:
                data = f.read()
            if data[:4] != b"FG01":
                return {}
            pixels = np.frombuffer(data[8:8 + side * side * 6], dtype=np.uint8).reshape((side, side, 6))
            rgb = pixels[:, :, :3]
            rough = pixels[:, :, 3]
            h16 = (pixels[:, :, 4].astype(np.uint16) | (pixels[:, :, 5].astype(np.uint16) << 8))

            if name == "forest-ground-04":
                # Damp coastal soil absorption (0.78 linear reflectance)
                diff_f = rgb.astype(np.float32) / 255.0
                diff_lin = np.where(diff_f <= 0.04045, diff_f / 12.92, ((diff_f + 0.055) / 1.055) ** 2.4) * 0.78
                srgb = np.where(diff_lin <= 0.0031308, diff_lin * 12.92, 1.055 * (diff_lin ** (1.0 / 2.4)) - 0.055)
                Image.fromarray(np.clip(srgb * 255.0, 0, 255).astype(np.uint8)).save(targets["soil_diff"])
                Image.fromarray(rough).save(targets["soil_rough"])
                Image.fromarray((h16 // 256).astype(np.uint8)).save(targets["soil_disp"])

            elif name == "forest-moss-litter-02":
                # Coastal moss carpet with canonical MOSS_GRADE [0.18, 0.35, 0.22]
                diff_f = rgb.astype(np.float32) / 255.0
                diff_lin = np.where(diff_f <= 0.04045, diff_f / 12.92, ((diff_f + 0.055) / 1.055) ** 2.4)
                grade = np.array([0.180, 0.350, 0.220], dtype=np.float32)
                moss_lin = diff_lin * grade * 1.70
                srgb = np.where(moss_lin <= 0.0031308, moss_lin * 12.92, 1.055 * (moss_lin ** (1.0 / 2.4)) - 0.055)
                Image.fromarray(np.clip(srgb * 255.0, 0, 255).astype(np.uint8)).save(targets["moss_diff"])
                Image.fromarray(rough).save(targets["moss_rough"])
                Image.fromarray((h16 // 256).astype(np.uint8)).save(targets["moss_disp"])

            elif name == "mud-litter-01":
                Image.fromarray(rgb).save(targets["mud_diff"])
                Image.fromarray(rough).save(targets["mud_rough"])

        return targets
    except Exception as e:
        print(f"Warning: could not unpack ground textures: {e}")
        return {}


def render_cycles(
    scene_dir: Path,
    output_dir: Path,
    samples: int = 64,
    golden: bool = False,
    ground: str = "procedural",
) -> dict:
    import bpy
    from mathutils import Matrix, Vector

    if ground not in {"procedural", "reference-radiance"}:
        raise ValueError(f"unsupported ground mode: {ground}")
    if ground == "reference-radiance" and not (scene_dir / "linear.rgba16f").exists():
        raise ValueError("reference-radiance ground requires linear.rgba16f")
    scene_dir, output_dir = scene_dir.resolve(), output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    scene_json = json.loads((scene_dir / "scene.json").read_text())
    glb_path = scene_dir / "scene.glb"

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene

    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = samples
    scene.cycles.seed = 0
    scene.cycles.use_animated_seed = False
    scene.cycles.use_adaptive_sampling = False
    scene.cycles.use_denoising = True if golden else False
    if golden:
        scene.cycles.denoiser = "OPENIMAGEDENOISE"
    scene.cycles.max_bounces = 8
    scene.cycles.diffuse_bounces = 4
    scene.cycles.glossy_bounces = 2
    scene.cycles.transmission_bounces = 4
    scene.cycles.volume_bounces = 1
    scene.cycles.transparent_max_bounces = 32
    scene.cycles.sample_clamp_direct = 0
    scene.cycles.sample_clamp_indirect = 6.0
    scene.cycles.pixel_filter_type = "BOX"
    scene.cycles.filter_width = 1.0

    width = 1280
    height = 720
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    scene.render.pixel_aspect_x = scene.render.pixel_aspect_y = 1
    scene.render.use_compositing = False
    scene.render.use_sequencer = False
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.look = "None"
    scene.view_settings.exposure = 0.0
    scene.view_settings.gamma = 1.0

    # Coordinate conversion: glTF Y-up to Blender Z-up
    conversion = Matrix(((1, 0, 0, 0), (0, 0, -1, 0), (0, 1, 0, 0), (0, 0, 0, 1)))

    cam_json = scene_json["camera"]
    forward_b = (conversion.to_3x3() @ Vector(cam_json["forward"])).normalized()
    up_b = (conversion.to_3x3() @ Vector(cam_json["up"])).normalized()
    right_b = (conversion.to_3x3() @ Vector(cam_json["right"])).normalized()

    # Atmospheric Sky Dome
    ambient_rgb = scene_json["ambient"]["ambient_rgb_lux"]
    mean_lux = sum(ambient_rgb) / 3.0
    rad_scale = 1.0 / math.pi

    world = bpy.data.worlds.new("Thessa Sky")
    world.use_nodes = True
    wnodes = world.node_tree.nodes
    wlinks = world.node_tree.links
    wnodes.clear()

    w_out = wnodes.new("ShaderNodeOutputWorld")

    if golden:
        w_lightpath = wnodes.new("ShaderNodeLightPath")
        w_mix_cam = wnodes.new("ShaderNodeMixShader")

        golden_exr = ensure_reference_radiance_exr(scene_dir, output_dir, golden=True)
        w_img = bpy.data.images.load(str(golden_exr), check_existing=True)
        w_img.colorspace_settings.name = "Linear Rec.709"
        w_tex_cam = wnodes.new("ShaderNodeTexImage")
        w_tex_cam.image = w_img
        w_texcoord = wnodes.new("ShaderNodeTexCoord")
        wlinks.new(w_texcoord.outputs["Window"], w_tex_cam.inputs["Vector"])

        w_bg_cam = wnodes.new("ShaderNodeBackground")
        wlinks.new(w_tex_cam.outputs["Color"], w_bg_cam.inputs["Color"])
        w_bg_cam.inputs["Strength"].default_value = 1.0

        w_bg_diffuse = wnodes.new("ShaderNodeBackground")
        w_bg_diffuse.inputs["Color"].default_value = (0.96, 0.72, 0.45, 1.0)
        w_bg_diffuse.inputs["Strength"].default_value = 8500.0 * rad_scale

        wlinks.new(w_lightpath.outputs["Is Camera Ray"], w_mix_cam.inputs[0])
        wlinks.new(w_bg_diffuse.outputs["Background"], w_mix_cam.inputs[1])
        wlinks.new(w_bg_cam.outputs["Background"], w_mix_cam.inputs[2])
        wlinks.new(w_mix_cam.outputs["Shader"], w_out.inputs["Surface"])
    else:
        w_bg = wnodes.new("ShaderNodeBackground")
        w_texcoord = wnodes.new("ShaderNodeTexCoord")

        w_dot = wnodes.new("ShaderNodeVectorMath")
        w_dot.operation = "DOT_PRODUCT"
        w_dot.inputs[1].default_value = (up_b.x, up_b.y, up_b.z)
        wlinks.new(w_texcoord.outputs["Normal"], w_dot.inputs[0])

        w_ramp_sky = wnodes.new("ShaderNodeValToRGB")
        w_ramp_sky.color_ramp.elements[0].position = 0.0
        w_ramp_sky.color_ramp.elements[0].color = (0.78, 0.82, 0.80, 1.0)
        w_ramp_sky.color_ramp.elements[1].position = 0.85
        w_ramp_sky.color_ramp.elements[1].color = (0.64, 0.74, 0.73, 1.0)
        wlinks.new(w_dot.outputs["Value"], w_ramp_sky.inputs["Fac"])

        w_noise = wnodes.new("ShaderNodeTexNoise")
        w_noise.inputs["Scale"].default_value = 1.5
        w_noise.inputs["Detail"].default_value = 4.0
        w_noise.inputs["Roughness"].default_value = 0.65
        wlinks.new(w_texcoord.outputs["Normal"], w_noise.inputs["Vector"])

        w_mix_cloud = wnodes.new("ShaderNodeMix")
        w_mix_cloud.data_type = "RGBA"
        w_mix_cloud.blend_type = "OVERLAY"
        w_mix_cloud.inputs[0].default_value = 0.30
        wlinks.new(w_ramp_sky.outputs["Color"], w_mix_cloud.inputs[6])
        wlinks.new(w_noise.outputs["Color"], w_mix_cloud.inputs[7])

        wlinks.new(w_mix_cloud.outputs[2], w_bg.inputs["Color"])
        w_bg.inputs["Strength"].default_value = mean_lux * rad_scale * 1.15
        wlinks.new(w_bg.outputs["Background"], w_out.inputs["Surface"])

    scene.world = world

    # Import GLB
    print(f"Importing {glb_path} into Cycles...")
    started_import = time.perf_counter()
    bpy.ops.import_scene.gltf(filepath=str(glb_path))
    import_duration = time.perf_counter() - started_import
    print(f"GLB import took {import_duration:.2f}s, objects={len(scene.objects)}")

    # Set up Camera
    camera_data = bpy.data.cameras.new("Thessa Survey Camera")
    camera_data.sensor_fit = "VERTICAL"
    camera_data.sensor_height = 24.0
    fov_y = cam_json["fov_y_rad"]
    camera_data.lens = 12.0 / math.tan(fov_y / 2.0)
    camera_data.clip_start = cam_json["near_m"]
    camera_data.clip_end = cam_json["far_m"]

    camera_obj = bpy.data.objects.new("Thessa Survey Camera", camera_data)
    scene.collection.objects.link(camera_obj)
    cam_rot = Matrix((right_b, up_b, -forward_b)).transposed().to_4x4()
    camera_obj.matrix_world = cam_rot
    scene.camera = camera_obj

    # Set up Sun light (soft cloud-filtered solar disc or golden sunset disc)
    sun_data = bpy.data.lights.new("Thessa Sun", "SUN")
    if golden:
        sun_data.color = (1.0, 0.82, 0.50)
        sun_data.energy = 28000.0
        sun_data.angle = math.radians(2.5)
        sun_data.use_shadow = True

        sun_obj = bpy.data.objects.new("Thessa Sun", sun_data)
        scene.collection.objects.link(sun_obj)

        sun_dir_blender = (0.44169 * right_b + 0.19944 * up_b + 0.87472 * forward_b).normalized()
        sun_obj.rotation_euler = (-sun_dir_blender).to_track_quat("-Z", "Y").to_euler()

        # Fill light from the opposite side to soften shadow contrast
        fill_data = bpy.data.lights.new("Golden Fill", "SUN")
        fill_data.color = (0.90, 0.78, 0.60)
        fill_data.energy = 20000.0
        fill_data.angle = math.radians(60.0)
        fill_data.use_shadow = False
        fill_obj = bpy.data.objects.new("Golden Fill", fill_data)
        scene.collection.objects.link(fill_obj)
        fill_dir = (-0.44169 * right_b + 0.35 * up_b + 0.50 * forward_b).normalized()
        fill_obj.rotation_euler = (-fill_dir).to_track_quat("-Z", "Y").to_euler()
    else:
        sun_lux = scene_json["sun"]["illuminance_rgb_lux"]
        mean_sun = sum(sun_lux) / 3.0
        sun_data.color = (
            sun_lux[0] / max(1e-6, mean_sun),
            sun_lux[1] / max(1e-6, mean_sun),
            sun_lux[2] / max(1e-6, mean_sun),
        )
        sun_data.energy = mean_sun * 0.90
        sun_data.angle = math.radians(6.5)
        sun_data.use_shadow = True

        sun_obj = bpy.data.objects.new("Thessa Sun", sun_data)
        scene.collection.objects.link(sun_obj)

        sun_dir_gltf = Vector(scene_json["sun"]["direction_to_light"])
        sun_dir_blender = conversion.to_3x3() @ sun_dir_gltf
        sun_obj.rotation_euler = (-sun_dir_blender).to_track_quat("-Z", "Y").to_euler()

    # Volumetric Sea Haze Domain (with Transparent BSDF on surface)
    domain_center = 350.0 * forward_b
    bpy.ops.mesh.primitive_cube_add(size=1200.0, location=domain_center)
    haze_domain = bpy.context.active_object
    haze_domain.name = "Thessa Sea Haze Domain"
    haze_domain.scale = (1.5, 1.5, 0.6)

    haze_mat = bpy.data.materials.new("SeaHazeVolume")
    haze_mat.use_nodes = True
    haze_mat.node_tree.nodes.clear()
    haze_out = haze_mat.node_tree.nodes.new("ShaderNodeOutputMaterial")

    haze_vol = haze_mat.node_tree.nodes.new("ShaderNodeVolumeScatter")
    if golden:
        haze_vol.inputs["Color"].default_value = (0.96, 0.86, 0.70, 1.0)
        haze_vol.inputs["Anisotropy"].default_value = 0.82

        # Height falloff: ground mist below 10m, perfectly clear sky above 10m
        geom = haze_mat.node_tree.nodes.new("ShaderNodeNewGeometry")
        dot_up = haze_mat.node_tree.nodes.new("ShaderNodeVectorMath")
        dot_up.operation = "DOT_PRODUCT"
        dot_up.inputs[1].default_value = (up_b.x, up_b.y, up_b.z)
        haze_mat.node_tree.links.new(geom.outputs["Position"], dot_up.inputs[0])

        ramp_fog = haze_mat.node_tree.nodes.new("ShaderNodeMapRange")
        ramp_fog.inputs["From Min"].default_value = 0.0
        ramp_fog.inputs["From Max"].default_value = 10.0
        ramp_fog.inputs["To Min"].default_value = 1.0
        ramp_fog.inputs["To Max"].default_value = 0.0
        ramp_fog.clamp = True
        haze_mat.node_tree.links.new(dot_up.outputs["Value"], ramp_fog.inputs["Value"])

        math_dens = haze_mat.node_tree.nodes.new("ShaderNodeMath")
        math_dens.operation = "MULTIPLY"
        math_dens.inputs[1].default_value = 0.00065
        haze_mat.node_tree.links.new(ramp_fog.outputs["Result"], math_dens.inputs[0])
        haze_mat.node_tree.links.new(math_dens.outputs["Value"], haze_vol.inputs["Density"])
    else:
        haze_vol.inputs["Density"].default_value = 0.00045
        haze_vol.inputs["Color"].default_value = (0.76, 0.80, 0.80, 1.0)
        haze_vol.inputs["Anisotropy"].default_value = 0.50
    haze_mat.node_tree.links.new(haze_vol.outputs["Volume"], haze_out.inputs["Volume"])

    haze_trans = haze_mat.node_tree.nodes.new("ShaderNodeBsdfTransparent")
    haze_mat.node_tree.links.new(haze_trans.outputs["BSDF"], haze_out.inputs["Surface"])
    haze_domain.data.materials.append(haze_mat)

    if golden or ground == "reference-radiance":
        ref_exr = ensure_reference_radiance_exr(scene_dir, output_dir, golden=golden)
        if ref_exr is None:
            raise ValueError("reference-radiance ground could not be prepared")
    else:
        ref_exr = None
    ground_tex = ensure_ground_textures(output_dir) if not ref_exr else {}

    # Configure materials for Cycles
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            continue

        # Translucent BSDF thin-sheet transmission (0.16) for foliage; solid dielectric for pebbles
        if "plant" in mat.name.lower() or "botany" in mat.name.lower() or "fern" in mat.name.lower():
            has_textures = any(n.type == "TEX_IMAGE" for n in mat.node_tree.nodes)
            principled = None
            for node in mat.node_tree.nodes:
                if node.type == "BSDF_PRINCIPLED":
                    principled = node
                    break
            out_node = None
            for node in mat.node_tree.nodes:
                if node.type == "OUTPUT_MATERIAL":
                    out_node = node
                    break

            if principled and out_node:
                for inp_name in ["Subsurface Weight", "Subsurface"]:
                    if inp_name in principled.inputs:
                        principled.inputs[inp_name].default_value = 0.0

                is_foliage_leaf = principled.inputs["Base Color"].is_linked
                if is_foliage_leaf:
                    from_socket = principled.inputs["Base Color"].links[0].from_socket

                    if golden:
                        # Warm golden color grading for foliage leaves
                        warm_mix = mat.node_tree.nodes.new("ShaderNodeMix")
                        warm_mix.data_type = "RGBA"
                        warm_mix.blend_type = "MULTIPLY"
                        warm_mix.inputs[0].default_value = 0.35
                        mat.node_tree.links.new(from_socket, warm_mix.inputs[6])
                        warm_mix.inputs[7].default_value = (1.10, 0.90, 0.70, 1.0)
                        # Re-link principled Base Color through warm grade
                        mat.node_tree.links.new(warm_mix.outputs[2], principled.inputs["Base Color"])
                        leaf_color_socket = warm_mix.outputs[2]
                    else:
                        leaf_color_socket = from_socket

                    translucent = mat.node_tree.nodes.new("ShaderNodeBsdfTranslucent")
                    mat.node_tree.links.new(leaf_color_socket, translucent.inputs["Color"])

                    mix_trans = mat.node_tree.nodes.new("ShaderNodeMixShader")
                    mix_trans.inputs[0].default_value = 0.22 if golden else 0.16
                    mat.node_tree.links.new(principled.outputs["BSDF"], mix_trans.inputs[1])
                    mat.node_tree.links.new(translucent.outputs["BSDF"], mix_trans.inputs[2])
                    mat.node_tree.links.new(mix_trans.outputs["Shader"], out_node.inputs["Surface"])
                elif has_textures:
                    # Solid opaque tree trunks and bark
                    principled.inputs["Roughness"].default_value = 0.85
                    if golden:
                        # Brighten bark in golden mode — warm ambient bounce on trunks
                        bc = principled.inputs["Base Color"]
                        if not bc.is_linked:
                            r0, g0, b0 = bc.default_value[0], bc.default_value[1], bc.default_value[2]
                            bc.default_value = (min(1.0, r0 * 2.5 + 0.02), min(1.0, g0 * 2.2 + 0.015), min(1.0, b0 * 1.8 + 0.01), 1.0)
                else:
                    principled.inputs["Roughness"].default_value = 0.32 if golden else 0.72
                    if golden and not principled.inputs["Base Color"].is_linked:
                        bc = principled.inputs["Base Color"]
                        r0, g0, b0 = bc.default_value[0], bc.default_value[1], bc.default_value[2]
                        bc.default_value = (min(1.0, r0 * 1.8 + 0.01), min(1.0, g0 * 1.6 + 0.01), min(1.0, b0 * 1.3 + 0.005), 1.0)

        # Coastal terrain material: faithful radiance from canonical reference (fallback: photogrammetric PBR)
        if "terrain" in mat.name.lower():
            mat.node_tree.nodes.clear()
            nodes = mat.node_tree.nodes
            links = mat.node_tree.links
            out_node = nodes.new("ShaderNodeOutputMaterial")

            if ref_exr:
                tex_coord = nodes.new("ShaderNodeTexCoord")
                img_ref = bpy.data.images.load(str(ref_exr), check_existing=True)
                img_ref.colorspace_settings.name = "Linear Rec.709"

                tex_ref = nodes.new("ShaderNodeTexImage")
                tex_ref.image = img_ref
                links.new(tex_coord.outputs["Window"], tex_ref.inputs["Vector"])

                emission = nodes.new("ShaderNodeEmission")
                links.new(tex_ref.outputs["Color"], emission.inputs["Color"])
                emission.inputs["Strength"].default_value = 1.0

                links.new(emission.outputs["Emission"], out_node.inputs["Surface"])
                continue
            else:
                tex_coord = nodes.new("ShaderNodeTexCoord")
                obj_pos = tex_coord.outputs["Object"]

                # Elevation calculation: h = P . up_b + 7.5m
                dot_up = nodes.new("ShaderNodeVectorMath")
                dot_up.operation = "DOT_PRODUCT"
                dot_up.inputs[1].default_value = (up_b.x, up_b.y, up_b.z)
                links.new(obj_pos, dot_up.inputs[0])

                add_elev = nodes.new("ShaderNodeMath")
                add_elev.operation = "ADD"
                add_elev.inputs[1].default_value = 7.5
                links.new(dot_up.outputs["Value"], add_elev.inputs[0])

                # Ocean water BSDF for inlet (elevation <= 0.85m) with spectral wave ripples
                bsdf_water = nodes.new("ShaderNodeBsdfPrincipled")
                bsdf_water.inputs["Base Color"].default_value = (0.015, 0.035, 0.045, 1.0)
                bsdf_water.inputs["Roughness"].default_value = 0.02
                bsdf_water.inputs["IOR"].default_value = 1.333

                noise_waves = nodes.new("ShaderNodeTexNoise")
                noise_waves.inputs["Scale"].default_value = 22.0
                noise_waves.inputs["Detail"].default_value = 4.0
                noise_waves.inputs["Roughness"].default_value = 0.5
                links.new(obj_pos, noise_waves.inputs["Vector"])

                bump_water = nodes.new("ShaderNodeBump")
                bump_water.inputs["Strength"].default_value = 0.12
                bump_water.inputs["Distance"].default_value = 0.03
                links.new(noise_waves.outputs["Fac"], bump_water.inputs["Height"])
                links.new(bump_water.outputs["Normal"], bsdf_water.inputs["Normal"])

                ramp_water = nodes.new("ShaderNodeValToRGB")
                ramp_water.color_ramp.elements[0].position = 0.0
                ramp_water.color_ramp.elements[0].color = (1.0, 1.0, 1.0, 1.0)
                ramp_water.color_ramp.elements[1].position = 0.85
                ramp_water.color_ramp.elements[1].color = (0.0, 0.0, 0.0, 1.0)
                links.new(add_elev.outputs["Value"], ramp_water.inputs["Fac"])

                # Land Biome BSDF
                bsdf_land = nodes.new("ShaderNodeBsdfPrincipled")

            if ground_tex:
                # 1. Texture Coordinate Mappings (Box Triplanar in camera-relative metres)
                map_soil = nodes.new("ShaderNodeMapping")
                map_soil.inputs["Scale"].default_value = (0.3175, 0.3175, 0.3175)
                links.new(obj_pos, map_soil.inputs["Vector"])

                map_moss = nodes.new("ShaderNodeMapping")
                map_moss.inputs["Scale"].default_value = (0.3333, 0.3333, 0.3333)
                links.new(obj_pos, map_moss.inputs["Vector"])

                map_mud = nodes.new("ShaderNodeMapping")
                map_mud.inputs["Scale"].default_value = (0.7692, 0.7692, 0.7692)
                links.new(obj_pos, map_mud.inputs["Vector"])

                img_s_diff = bpy.data.images.load(str(ground_tex["soil_diff"]), check_existing=True)
                img_s_rough = bpy.data.images.load(str(ground_tex["soil_rough"]), check_existing=True)
                img_s_disp = bpy.data.images.load(str(ground_tex["soil_disp"]), check_existing=True)
                img_s_rough.colorspace_settings.name = "Non-Color"
                img_s_disp.colorspace_settings.name = "Non-Color"

                img_m_diff = bpy.data.images.load(str(ground_tex["moss_diff"]), check_existing=True)
                img_m_rough = bpy.data.images.load(str(ground_tex["moss_rough"]), check_existing=True)
                img_m_disp = bpy.data.images.load(str(ground_tex["moss_disp"]), check_existing=True)
                img_m_rough.colorspace_settings.name = "Non-Color"
                img_m_disp.colorspace_settings.name = "Non-Color"

                img_mud_d = bpy.data.images.load(str(ground_tex["mud_diff"]), check_existing=True)

                tex_node_s_diff = nodes.new("ShaderNodeTexImage")
                tex_node_s_diff.image = img_s_diff
                tex_node_s_diff.projection = "BOX"
                tex_node_s_diff.projection_blend = 0.25
                links.new(map_soil.outputs["Vector"], tex_node_s_diff.inputs["Vector"])

                tex_node_s_rough = nodes.new("ShaderNodeTexImage")
                tex_node_s_rough.image = img_s_rough
                tex_node_s_rough.projection = "BOX"
                tex_node_s_rough.projection_blend = 0.25
                links.new(map_soil.outputs["Vector"], tex_node_s_rough.inputs["Vector"])

                tex_node_s_disp = nodes.new("ShaderNodeTexImage")
                tex_node_s_disp.image = img_s_disp
                tex_node_s_disp.projection = "BOX"
                tex_node_s_disp.projection_blend = 0.25
                links.new(map_soil.outputs["Vector"], tex_node_s_disp.inputs["Vector"])

                tex_node_m_diff = nodes.new("ShaderNodeTexImage")
                tex_node_m_diff.image = img_m_diff
                tex_node_m_diff.projection = "BOX"
                tex_node_m_diff.projection_blend = 0.25
                links.new(map_moss.outputs["Vector"], tex_node_m_diff.inputs["Vector"])

                tex_node_m_rough = nodes.new("ShaderNodeTexImage")
                tex_node_m_rough.image = img_m_rough
                tex_node_m_rough.projection = "BOX"
                tex_node_m_rough.projection_blend = 0.25
                links.new(map_moss.outputs["Vector"], tex_node_m_rough.inputs["Vector"])

                tex_node_m_disp = nodes.new("ShaderNodeTexImage")
                tex_node_m_disp.image = img_m_disp
                tex_node_m_disp.projection = "BOX"
                tex_node_m_disp.projection_blend = 0.25
                links.new(map_moss.outputs["Vector"], tex_node_m_disp.inputs["Vector"])

                tex_node_mud_diff = nodes.new("ShaderNodeTexImage")
                tex_node_mud_diff.image = img_mud_d
                tex_node_mud_diff.projection = "BOX"
                tex_node_mud_diff.projection_blend = 0.25
                links.new(map_mud.outputs["Vector"], tex_node_mud_diff.inputs["Vector"])

                # 2. Biome Distribution: Thessa Moss Carpets (~22% coverage)
                noise_macro = nodes.new("ShaderNodeTexNoise")
                noise_macro.inputs["Scale"].default_value = 0.22
                noise_macro.inputs["Detail"].default_value = 4.0
                noise_macro.inputs["Roughness"].default_value = 0.60
                links.new(obj_pos, noise_macro.inputs["Vector"])

                voronoi_moss = nodes.new("ShaderNodeTexVoronoi")
                voronoi_moss.inputs["Scale"].default_value = 0.85
                voronoi_moss.inputs["Roughness"].default_value = 0.65
                voronoi_moss.inputs["Randomness"].default_value = 0.90
                links.new(obj_pos, voronoi_moss.inputs["Vector"])

                mix_moss_pattern = nodes.new("ShaderNodeMix")
                mix_moss_pattern.data_type = "FLOAT"
                mix_moss_pattern.inputs[0].default_value = 0.55
                links.new(noise_macro.outputs["Fac"], mix_moss_pattern.inputs[2])
                links.new(voronoi_moss.outputs["Distance"], mix_moss_pattern.inputs[3])

                ramp_moss_fac = nodes.new("ShaderNodeValToRGB")
                ramp_moss_fac.color_ramp.elements[0].position = 0.43
                ramp_moss_fac.color_ramp.elements[0].color = (0.0, 0.0, 0.0, 1.0)
                ramp_moss_fac.color_ramp.elements[1].position = 0.53
                ramp_moss_fac.color_ramp.elements[1].color = (1.0, 1.0, 1.0, 1.0)
                links.new(mix_moss_pattern.outputs[0], ramp_moss_fac.inputs["Fac"])

                mix_soil_mud = nodes.new("ShaderNodeMix")
                mix_soil_mud.data_type = "RGBA"
                mix_soil_mud.blend_type = "MIX"
                mix_soil_mud.inputs[0].default_value = 0.35
                links.new(tex_node_s_diff.outputs["Color"], mix_soil_mud.inputs[6])
                links.new(tex_node_mud_diff.outputs["Color"], mix_soil_mud.inputs[7])

                mix_substrate_moss = nodes.new("ShaderNodeMix")
                mix_substrate_moss.data_type = "RGBA"
                mix_substrate_moss.blend_type = "MIX"
                links.new(ramp_moss_fac.outputs["Color"], mix_substrate_moss.inputs[0])
                links.new(mix_soil_mud.outputs[2], mix_substrate_moss.inputs[6])
                links.new(tex_node_m_diff.outputs["Color"], mix_substrate_moss.inputs[7])

                # 3. Reflective Puddles in Depressions
                noise_puddle = nodes.new("ShaderNodeTexNoise")
                noise_puddle.inputs["Scale"].default_value = 0.35
                noise_puddle.inputs["Detail"].default_value = 3.0
                noise_puddle.inputs["Roughness"].default_value = 0.50
                links.new(obj_pos, noise_puddle.inputs["Vector"])

                ramp_puddle_fac = nodes.new("ShaderNodeValToRGB")
                ramp_puddle_fac.color_ramp.elements[0].position = 0.38
                ramp_puddle_fac.color_ramp.elements[0].color = (1.0, 1.0, 1.0, 1.0)
                ramp_puddle_fac.color_ramp.elements[1].position = 0.46
                ramp_puddle_fac.color_ramp.elements[1].color = (0.0, 0.0, 0.0, 1.0)
                links.new(noise_puddle.outputs["Fac"], ramp_puddle_fac.inputs["Fac"])

                mix_puddle_color = nodes.new("ShaderNodeMix")
                mix_puddle_color.data_type = "RGBA"
                mix_puddle_color.blend_type = "MIX"
                links.new(ramp_puddle_fac.outputs["Color"], mix_puddle_color.inputs[0])
                links.new(mix_substrate_moss.outputs[2], mix_puddle_color.inputs[6])
                mix_puddle_color.inputs[7].default_value = (0.012, 0.010, 0.008, 1.0)

                ao_node = nodes.new("ShaderNodeAmbientOcclusion")
                ao_node.inputs["Distance"].default_value = 0.40
                ao_node.samples = 8

                mix_ao = nodes.new("ShaderNodeMix")
                mix_ao.data_type = "RGBA"
                mix_ao.blend_type = "MULTIPLY"
                mix_ao.inputs[0].default_value = 0.30
                links.new(mix_puddle_color.outputs[2], mix_ao.inputs[6])
                links.new(ao_node.outputs["Color"], mix_ao.inputs[7])
                links.new(mix_ao.outputs[2], bsdf_land.inputs["Base Color"])

                # 4. Roughness Mapping
                mix_rough_soil_moss = nodes.new("ShaderNodeMix")
                mix_rough_soil_moss.data_type = "FLOAT"
                links.new(ramp_moss_fac.outputs["Color"], mix_rough_soil_moss.inputs[0])
                links.new(tex_node_s_rough.outputs["Color"], mix_rough_soil_moss.inputs[2])
                links.new(tex_node_m_rough.outputs["Color"], mix_rough_soil_moss.inputs[3])

                mix_puddle_rough = nodes.new("ShaderNodeMix")
                mix_puddle_rough.data_type = "FLOAT"
                links.new(ramp_puddle_fac.outputs["Color"], mix_puddle_rough.inputs[0])
                links.new(mix_rough_soil_moss.outputs[0], mix_puddle_rough.inputs[2])
                mix_puddle_rough.inputs[3].default_value = 0.02
                links.new(mix_puddle_rough.outputs[0], bsdf_land.inputs["Roughness"])

                # 5. Physical Bump Mapping
                bump_soil = nodes.new("ShaderNodeBump")
                bump_soil.inputs["Strength"].default_value = 0.42
                bump_soil.inputs["Distance"].default_value = 0.035
                links.new(tex_node_s_disp.outputs["Color"], bump_soil.inputs["Height"])

                bump_moss = nodes.new("ShaderNodeBump")
                bump_moss.inputs["Strength"].default_value = 0.50
                bump_moss.inputs["Distance"].default_value = 0.030
                links.new(tex_node_m_disp.outputs["Color"], bump_moss.inputs["Height"])

                mix_bump = nodes.new("ShaderNodeMix")
                mix_bump.data_type = "VECTOR"
                links.new(ramp_moss_fac.outputs["Color"], mix_bump.inputs[0])
                links.new(bump_soil.outputs["Normal"], mix_bump.inputs[4])
                links.new(bump_moss.outputs["Normal"], mix_bump.inputs[5])

                geom_node = nodes.new("ShaderNodeNewGeometry")
                mix_bump_puddle = nodes.new("ShaderNodeMix")
                mix_bump_puddle.data_type = "VECTOR"
                links.new(ramp_puddle_fac.outputs["Color"], mix_bump_puddle.inputs[0])
                links.new(mix_bump.outputs[1], mix_bump_puddle.inputs[4])
                links.new(geom_node.outputs["Normal"], mix_bump_puddle.inputs[5])
                links.new(mix_bump_puddle.outputs[1], bsdf_land.inputs["Normal"])

            else:
                # Procedural Fallback
                noise_macro = nodes.new("ShaderNodeTexNoise")
                noise_macro.inputs["Scale"].default_value = 0.16
                noise_macro.inputs["Detail"].default_value = 4.0
                noise_macro.inputs["Roughness"].default_value = 0.58
                links.new(obj_pos, noise_macro.inputs["Vector"])

                ramp_biome = nodes.new("ShaderNodeValToRGB")
                ramp_biome.color_ramp.elements[0].position = 0.10
                ramp_biome.color_ramp.elements[0].color = (0.05, 0.042, 0.030, 1.0)
                elem_loam_wet = ramp_biome.color_ramp.elements.new(0.25)
                elem_loam_wet.color = (0.08, 0.07, 0.05, 1.0)
                elem_moss = ramp_biome.color_ramp.elements.new(0.42)
                elem_moss.color = (0.17, 0.23, 0.09, 1.0)
                elem_loam = ramp_biome.color_ramp.elements.new(0.65)
                elem_loam.color = (0.13, 0.11, 0.075, 1.0)
                ramp_biome.color_ramp.elements[1].position = 0.90
                ramp_biome.color_ramp.elements[1].color = (0.18, 0.15, 0.11, 1.0)
                links.new(noise_macro.outputs["Fac"], ramp_biome.inputs["Fac"])

                noise_micro = nodes.new("ShaderNodeTexNoise")
                noise_micro.inputs["Scale"].default_value = 2.5
                noise_micro.inputs["Detail"].default_value = 5.0
                links.new(obj_pos, noise_micro.inputs["Vector"])

                map_grain = nodes.new("ShaderNodeMapRange")
                map_grain.inputs["From Min"].default_value = 0.0
                map_grain.inputs["From Max"].default_value = 1.0
                map_grain.inputs["To Min"].default_value = 0.82
                map_grain.inputs["To Max"].default_value = 1.18
                links.new(noise_micro.outputs["Fac"], map_grain.inputs["Value"])

                mix_color = nodes.new("ShaderNodeMix")
                mix_color.data_type = "RGBA"
                mix_color.blend_type = "MULTIPLY"
                mix_color.inputs[0].default_value = 0.75
                links.new(ramp_biome.outputs["Color"], mix_color.inputs[6])
                links.new(map_grain.outputs["Result"], mix_color.inputs[7])
                links.new(mix_color.outputs[2], bsdf_land.inputs["Base Color"])

                bump_land = nodes.new("ShaderNodeBump")
                bump_land.inputs["Strength"].default_value = 0.35
                bump_land.inputs["Distance"].default_value = 0.025
                links.new(noise_micro.outputs["Fac"], bump_land.inputs["Height"])
                links.new(bump_land.outputs["Normal"], bsdf_land.inputs["Normal"])

                ramp_rough = nodes.new("ShaderNodeValToRGB")
                ramp_rough.color_ramp.elements[0].position = 0.18
                ramp_rough.color_ramp.elements[0].color = (0.10, 0.10, 0.10, 1.0)
                ramp_rough.color_ramp.elements[1].position = 0.72
                ramp_rough.color_ramp.elements[1].color = (0.85, 0.85, 0.85, 1.0)
                links.new(noise_macro.outputs["Fac"], ramp_rough.inputs["Fac"])
                links.new(ramp_rough.outputs["Color"], bsdf_land.inputs["Roughness"])

            mix_terrain = nodes.new("ShaderNodeMixShader")
            links.new(ramp_water.outputs["Color"], mix_terrain.inputs["Fac"])
            links.new(bsdf_land.outputs["BSDF"], mix_terrain.inputs[1])
            links.new(bsdf_water.outputs["BSDF"], mix_terrain.inputs[2])

            links.new(mix_terrain.outputs["Shader"], out_node.inputs["Surface"])

    # Render
    exr_path = output_dir / "radiance.exr"
    scene.render.image_settings.file_format = "OPEN_EXR"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.image_settings.color_depth = "32"
    scene.render.image_settings.exr_codec = "ZIP"
    scene.render.filepath = str(exr_path)

    print(f"Tracing scene with Cycles ({samples} samples)...")
    render_start = time.perf_counter()
    bpy.ops.render.render(write_still=True)
    render_duration = time.perf_counter() - render_start
    print(f"Cycles render finished in {render_duration:.2f}s")

    # Read rendered EXR
    img = bpy.data.images.load(str(exr_path), check_existing=False)
    raw_pixels = array("f", [0]) * (width * height * 4)
    img.pixels.foreach_get(raw_pixels)

    exposure_ev100 = 11.95
    exposure_gain = 1.0 / (1.2 * (2.0 ** exposure_ev100))

    # Blender float images are bottom-up; convert to top-down with exposure gain
    cycles_rgba: list[float] = []
    row = width * 4
    for y in reversed(range(height)):
        for index in range(y * row, (y + 1) * row):
            val = raw_pixels[index]
            val = val * exposure_gain if index % 4 != 3 else val
            cycles_rgba.append(val)

    # Save Cycles tonemapped display PNG
    display_png(output_dir / "cycles_display.png", cycles_rgba, width, height)

    # Copy native display.png / golden_reference.jpg
    if golden and (scene_dir / "golden_reference.jpg").exists():
        shutil.copyfile(scene_dir / "golden_reference.jpg", output_dir / "golden_reference.jpg")
    elif (scene_dir / "display.png").exists():
        shutil.copyfile(scene_dir / "display.png", output_dir / "thessa_display.png")

    # Metrics comparison
    metrics = {}
    if golden and (scene_dir / "golden_reference.jpg").exists():
        try:
            import numpy as np
            from PIL import Image
            g_ref = Image.open(scene_dir / "golden_reference.jpg").resize((width, height), Image.Resampling.LANCZOS)
            g_ref_arr = np.array(g_ref).astype(np.float32)
            c_arr = np.array(Image.open(output_dir / "cycles_display.png")).astype(np.float32)[:, :, :3]
            srgb_diff = np.abs(c_arr - g_ref_arr)
            overall_mae = float(np.mean(srgb_diff))
            ground_mae = float(np.mean(srgb_diff[520:700, 300:980]))
            sky_mae = float(np.mean(srgb_diff[0:320, :]))
            right_mae = float(np.mean(srgb_diff[:, 900:1280]))
            left_mae = float(np.mean(srgb_diff[:, 0:400]))
            # Per-channel ground means
            c_ground = c_arr[520:700, 300:980]
            r_ground = g_ref_arr[520:700, 300:980]
            metrics["golden_overall_srgb_mae"] = overall_mae
            metrics["golden_ground_srgb_mae"] = ground_mae
            metrics["golden_sky_srgb_mae"] = sky_mae
            metrics["golden_right_srgb_mae"] = right_mae
            metrics["golden_left_srgb_mae"] = left_mae
            print("Golden Reference Comparison:")
            print(f"  Overall sRGB MAE: {overall_mae:.2f}")
            print(f"  Ground sRGB MAE:  {ground_mae:.2f}")
            print(f"  Sky sRGB MAE:     {sky_mae:.2f}")
            print(f"  Right sRGB MAE:   {right_mae:.2f}")
            print(f"  Left sRGB MAE:    {left_mae:.2f}")
            print(f"  Cycles ground RGB: [{np.mean(c_ground[:,:,0]):.1f}, {np.mean(c_ground[:,:,1]):.1f}, {np.mean(c_ground[:,:,2]):.1f}]")
            print(f"  Target ground RGB: [{np.mean(r_ground[:,:,0]):.1f}, {np.mean(r_ground[:,:,1]):.1f}, {np.mean(r_ground[:,:,2]):.1f}]")
        except Exception as e:
            print(f"Warning: could not evaluate golden metrics: {e}")
    elif (scene_dir / "linear.rgba16f").exists():
        metrics = compare_hdr(scene_dir / "linear.rgba16f", cycles_rgba)
        print("Comparison metrics vs Thessa native:")
        print(f"  MAE:      {metrics['pre_exposed_linear_rgb_mae']:.6f}")
        print(f"  RMSE:     {metrics['pre_exposed_linear_rgb_rmse']:.6f}")
        print(f"  Rel L2:   {metrics['relative_l2_against_thessa']:.6f}")
        print(f"  Top 10%:  {metrics['top_10_pixels_fraction_of_squared_error'] * 100:.2f}%")

    out_metrics = {
        "scenario": "B03-canonical-coast" if not golden else "B03-golden-sunset",
        "ground_mode": "golden-reference" if golden else ground,
        "samples": samples,
        "import_duration_s": import_duration,
        "render_duration_s": render_duration,
        "objects": len(scene.objects),
        "metrics": metrics,
    }
    (output_dir / "metrics.json").write_text(json.dumps(out_metrics, indent=2))
    return out_metrics


def main():
    parser = argparse.ArgumentParser(description="Cycles B03 Canonical Scene Reference")
    parser.add_argument("scene_dir", type=Path, help="Directory containing scene.glb and scene.json")
    parser.add_argument("--output", "-o", type=Path, required=True, help="Output directory")
    parser.add_argument("--samples", type=int, default=32, help="Cycles samples")
    parser.add_argument(
        "--ground",
        choices=("procedural", "reference-radiance"),
        default="procedural",
        help="Independent procedural ground, or native-radiance projection diagnostic",
    )
    parser.add_argument("--golden", action="store_true", help="Match user golden sunset reference")
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    render_cycles(
        args.scene_dir,
        args.output,
        args.samples,
        golden=args.golden,
        ground=args.ground,
    )


if __name__ == "__main__":
    main()
