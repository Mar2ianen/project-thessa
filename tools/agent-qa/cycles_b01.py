"""Run with Blender --background --factory-startup --disable-autoexec --python-exit-code 1
--python tools/agent-qa/cycles_b01.py -- CAPTURE_DIRECTORY --output NEW_DIRECTORY.
Same-scene B01 diagnostic; numeric irradiance mapping is not spectral calibration.
"""
from __future__ import annotations

import argparse
from array import array
import heapq
import json
import math
from pathlib import Path
import shutil
import struct
import sys
import time
import tomllib
import zlib

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture import sha256, source_state, reference_identity, validate_capture

ROOT = Path(__file__).resolve().parents[2]
B01_ID = "B01-material-lab-v1"
INPUTS = ("scene.json", "scene.glb", "linear.rgba16f", "display.png", "scene.toml", "perf.json", "scene.LICENSE.txt")


def load_inputs(directory: Path) -> tuple[dict, dict, dict]:
    capture = json.loads((directory / "capture.json").read_text())
    for name in INPUTS:
        if sha256(directory / name) != capture["artifacts_sha256"][name]:
            raise ValueError(f"native capture artifact changed: {name}")
    manifest = tomllib.loads((directory / "scene.toml").read_text())
    label = capture["source"]["commit"] + ("-dirty" if capture["source"]["dirty"] else "")
    perf = json.loads((directory / "perf.json").read_text())
    validate_capture(perf, manifest, label)
    if perf["metadata"] != capture["metadata"]:
        raise ValueError("perf metadata differs from capture provenance")
    for name, actual in reference_identity(directory, manifest, perf).items():
        if actual != capture[name]:
            raise ValueError(f"reference identity changed: {name}")
    scene = json.loads((directory / "scene.json").read_text())
    values = capture["metadata"]["graphics"]["resolved"]
    graphics = tomllib.loads(values["requested_toml"])
    renderer = graphics["renderer"]
    temporal = graphics["temporal"]
    if (scene["id"] != B01_ID or scene["schema_version"] != 1
            or scene["environment"] != "none" or scene["ambient_rgb_lux"] != [0, 0, 0]
            or scene["illuminance_rgb_lux"] != [10000, 10000, 10000]
            or scene["coordinate_system"] != "right-handed, Y-up"
            or scene["units"] != "metres" or scene["dielectric_f0"] != 0.04
            or not renderer["hdr"] or renderer["auto_exposure"] or renderer["bloom"]
            or renderer["tone_mapping"] != "reinhard" or not renderer["specular_reflections"]
            or renderer["msaa_samples"] != 1 or renderer["resolution_scale"] != 1.0
            or renderer["surface_quality"] != "high" or graphics["shadows"]["enabled"]
            or renderer["ray_tracing"] != "off" or any(temporal[key] for key in ("taa", "motion_vectors", "jitter"))
            or not capture["linear_hdr"]["available"] or not capture["linear_hdr"]["pre_exposed"]
            or capture["linear_hdr"]["tonemapped"] or capture["linear_hdr"]["format"] != "RGBA16Float"):
        raise ValueError("only the frozen direct-light B01 HDR/Reinhard baseline is supported")
    lights = json.loads(values["capture_light_colors_lux"])
    directions = json.loads(values["capture_light_directions"])
    ambient = json.loads(values["capture_ambient_lux"])
    if (lights != [[10000, 10000, 10000, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
            or ambient != [0, 0, 0, 0] or float(values["capture_simulation_time_s"]) != 0
            or len(directions) != 3 or len(directions[0]) != 4
            or any(not math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-7)
                   for a, b in zip(directions[0][:3], scene["direction_to_light"]))):
        raise ValueError("captured lighting differs from the frozen scene")
    width, height = manifest["width"], manifest["height"]
    if not (1 <= width <= 1280 and 1 <= height <= 960):
        raise ValueError("diagnostic extent budget exceeded")
    if (directory / "linear.rgba16f").stat().st_size != width * height * 8:
        raise ValueError("native HDR payload size mismatch")
    gain = float(values["capture_exposure_gain"])
    if not math.isfinite(gain) or gain <= 0:
        raise ValueError("invalid native exposure")
    if not math.isclose(gain, 1 / (1.2 * 2 ** renderer["exposure_ev100"]), rel_tol=1e-6):
        raise ValueError("native exposure convention differs")
    return capture, scene, {"width": width, "height": height, "exposure_gain": gain}


def display_png(path: Path, rgba, width: int, height: int) -> None:
    if len(rgba) != width * height * 4 or any(not math.isfinite(v) for v in rgba):
        raise ValueError("invalid finite RGBA display payload")
    pixels = bytearray()
    for offset in range(0, len(rgba), 4):
        if offset % (width * 4) == 0:
            pixels.append(0)
        for value in rgba[offset:offset + 3]:
            x = min(65504, max(0, value))
            x = x / (1 + x)
            x = 12.92 * x if x <= 0.0031308 else 1.055 * x ** (1 / 2.4) - 0.055
            pixels.append(round(255 * x))
        pixels.append(255)

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
                     + chunk(b"sRGB", b"\x00") + chunk(b"IDAT", zlib.compress(pixels)) + chunk(b"IEND", b""))


def compare_hdr(native: Path, rgba) -> dict:
    data = native.read_bytes()
    if len(data) != len(rgba) * 2:
        raise ValueError("comparison HDR extent differs")
    squared_error = squared_reference = absolute_error = 0.0
    foreground = components = 0
    largest_pixel_errors = []
    for index, sample in enumerate(struct.iter_unpack("<4e", data)):
        candidate = rgba[index * 4:index * 4 + 4]
        if any(not math.isfinite(v) for v in (*sample, *candidate)):
            raise ValueError("non-finite HDR comparison input")
        if max(*sample[:3], *candidate[:3]) > 1e-8:
            foreground += 1
        pixel_error = 0.0
        for before, after in zip(sample[:3], candidate[:3]):
            error = before - after
            squared_error += error * error
            pixel_error += error * error
            squared_reference += before * before
            absolute_error += abs(error)
            components += 1
        heapq.heappush(largest_pixel_errors, pixel_error)
        if len(largest_pixel_errors) > 10:
            heapq.heappop(largest_pixel_errors)
    return {"rgb_components": components, "foreground_union_pixels": foreground,
            "pre_exposed_linear_rgb_mae": absolute_error / components,
            "pre_exposed_linear_rgb_rmse": math.sqrt(squared_error / components),
            "relative_l2_against_thessa": math.sqrt(squared_error / squared_reference) if squared_reference else None,
            "top_10_pixels_fraction_of_squared_error": sum(largest_pixel_errors) / squared_error if squared_error else 0,
            "interpretation": "Whole-frame diagnostic including AA/silhouette and BRDF differences; no pass threshold or parity claim."}


def render(directory: Path, output: Path, samples: int) -> dict:
    import bpy
    from mathutils import Matrix, Vector

    directory, output = directory.resolve(), output.resolve()
    if output == ROOT or ROOT in output.parents or output == directory or directory in output.parents:
        raise ValueError("use a new output directory outside the checkout and native capture")
    if not 1 <= samples <= 1024:
        raise ValueError("samples must be in 1..1024")
    adapter_source = source_state(ROOT)
    adapter_hash = sha256(Path(__file__))
    capture, definition, settings = load_inputs(directory)
    input_hash = sha256(directory / "capture.json")
    output.mkdir(parents=True, exist_ok=False)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = samples
    scene.cycles.seed = 0
    scene.cycles.use_animated_seed = False
    scene.cycles.use_adaptive_sampling = False
    scene.cycles.use_denoising = False
    scene.cycles.max_bounces = 1
    scene.cycles.diffuse_bounces = 1
    scene.cycles.glossy_bounces = 1
    scene.cycles.transmission_bounces = 0
    scene.cycles.volume_bounces = 0
    scene.cycles.transparent_max_bounces = 0
    scene.cycles.sample_clamp_direct = 0
    scene.cycles.sample_clamp_indirect = 0
    scene.cycles.pixel_filter_type = "BOX"
    scene.cycles.filter_width = 1
    scene.render.resolution_x = settings["width"]
    scene.render.resolution_y = settings["height"]
    scene.render.resolution_percentage = 100
    scene.render.pixel_aspect_x = scene.render.pixel_aspect_y = 1
    scene.render.use_compositing = False
    scene.render.use_sequencer = False
    scene.render.film_transparent = False
    scene.render.dither_intensity = 0
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.look = "None"
    scene.view_settings.exposure = 0
    scene.view_settings.gamma = 1
    scene.world = bpy.data.worlds.new("B01 black world")
    scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0
    bpy.ops.import_scene.gltf(filepath=str(directory / "scene.glb"))
    objects = [obj for obj in scene.objects if obj.type == "MESH"]
    triangles = 0
    for obj in objects:
        obj.data.calc_loop_triangles()
        triangles += len(obj.data.loop_triangles)
        obj.visible_shadow = False
        obj.visible_diffuse = False
        obj.visible_glossy = False
    if len(objects) != 11 or triangles != 22082:
        raise ValueError(f"imported workload changed: objects={len(objects)} triangles={triangles}")
    # glTF importer maps (x,y,z) to Blender (x,-z,y). Camera local axes agree.
    conversion = Matrix(((1, 0, 0, 0), (0, 0, -1, 0), (0, 1, 0, 0), (0, 0, 0, 1)))
    values = capture["metadata"]["graphics"]["resolved"]
    world = json.loads(values["capture_world_from_view"])
    camera_matrix = Matrix(tuple(tuple(world[column * 4 + row] for column in range(4)) for row in range(4)))
    camera = bpy.data.objects.new("B01 fixed camera", bpy.data.cameras.new("B01 fixed camera"))
    scene.collection.objects.link(camera)
    camera.matrix_world = conversion @ camera_matrix
    camera.data.sensor_fit = "VERTICAL"
    camera.data.sensor_height = 24
    projection = json.loads(values["capture_clip_from_view"])
    camera.data.lens = 12 * projection[5]
    camera.data.clip_start = definition["near_m"]
    camera.data.clip_end = definition["far_m"]
    scene.camera = camera
    sun_data = bpy.data.lights.new("B01 numeric-irradiance sun", "SUN")
    sun_data.normalize = True
    sun_data.energy = definition["illuminance_rgb_lux"][0]
    sun_data.color = (1, 1, 1)
    sun_data.angle = 0
    sun_data.use_shadow = False
    sun = bpy.data.objects.new("B01 numeric-irradiance sun", sun_data)
    scene.collection.objects.link(sun)
    direction = conversion.to_3x3() @ Vector(definition["direction_to_light"])
    sun.rotation_euler = (-direction).to_track_quat("-Z", "Y").to_euler()
    scene.render.image_settings.file_format = "OPEN_EXR"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.image_settings.color_depth = "32"
    scene.render.image_settings.exr_codec = "ZIP"
    scene.render.filepath = str(output / "radiance.exr")
    started = time.perf_counter()
    bpy.ops.render.render(write_still=True)
    render_wall = time.perf_counter() - started
    image = bpy.data.images.load(str(output / "radiance.exr"), check_existing=False)
    if list(image.size) != [settings["width"], settings["height"]]:
        raise ValueError("Cycles output extent differs")
    pixels = array("f", [0]) * (settings["width"] * settings["height"] * 4)
    image.pixels.foreach_get(pixels)
    # Blender float images are bottom-up; native readback and PNG are top-down.
    rgba = array("f")
    row = settings["width"] * 4
    for y in reversed(range(settings["height"])):
        for index in range(y * row, (y + 1) * row):
            value = pixels[index] * settings["exposure_gain"] if index % 4 != 3 else pixels[index]
            if not math.isfinite(value):
                raise ValueError("non-finite Cycles output")
            rgba.append(value)
    binary = array("f", rgba)
    if sys.byteorder != "little":
        binary.byteswap()
    (output / "linear.rgba32f").write_bytes(binary.tobytes())
    display_png(output / "display.png", rgba, settings["width"], settings["height"])
    shutil.copyfile(directory / "display.png", output / "thessa.png")
    metrics = compare_hdr(directory / "linear.rgba16f", rgba)
    for name in ("scene.glb", "scene.json", "scene.LICENSE.txt"):
        shutil.copyfile(directory / name, output / name)
    if sha256(directory / "capture.json") != input_hash or load_inputs(directory)[0] != capture:
        raise ValueError("native inputs changed during Cycles render")
    if source_state(ROOT) != adapter_source or sha256(Path(__file__)) != adapter_hash:
        raise ValueError("adapter source changed during capture")
    report = {"schema_version": 1, "scene_id": B01_ID, "adapter_source": adapter_source,
              "adapter_sha256": adapter_hash, "native_capture_sha256": input_hash,
              "native_source": capture["source"], "native_platform": capture["metadata"]["platform"],
              "native_build": capture["metadata"]["build"],
              "reference_geometry_sha256": capture["reference_geometry_sha256"],
              "reference_scene_manifest_sha256": capture["reference_scene_manifest_sha256"],
              "engine": {"name": "Blender Cycles", "version": bpy.app.version_string,
                         "build_hash": bpy.app.build_hash.decode(), "binary_sha256": sha256(Path(bpy.app.binary_path)),
                         "device": "CPU", "threads": scene.render.threads},
              "capture": {**settings, "samples": samples, "seed": 0, "adaptive_sampling": False,
                          "denoising": False, "max_bounces": 1, "shadows": False,
                          "filter": "BOX, width 1", "tone_mapping": "shared per-channel Reinhard then sRGB",
                          "color_management": "scene-linear EXR; no AgX/Standard view baked into comparison",
                          "loaded_exr_color_space": image.colorspace_settings.name,
                          "camera_world_matrix_blender": [list(row) for row in camera.matrix_world],
                          "camera_vertical_fov_rad": camera.data.angle_y, "sun_energy_numeric": sun_data.energy,
                          "sun_angle_rad": sun_data.angle, "triangles": triangles, "mesh_objects": len(objects)},
              "light_units": "Operational numeric mapping: 1 Thessa RGB lux input = 1 Cycles normalized SUN energy unit. No spectral/luminous-efficacy calibration; absolute photometric equivalence is unverified.",
              "differences": ["Cycles glTF Principled BSDF versus Thessa direct GGX/Lambert/Fresnel and derivative roughness filtering",
                              "Cycles path sampling/BOX pixel reconstruction versus native raster MSAA=1",
                              "Cycles float32 radiance versus Thessa pre-exposed float16 render target",
                              "CPU offline Cycles render versus GPU raster; durations are not a performance comparison"],
              "render_wall_s": render_wall, "metrics": metrics,
              "acceptance": {"physical": "not evaluated", "visual": "not evaluated",
                             "performance": "not comparable", "temporal": "not evaluated"},
              "artifacts_sha256": {p.name: sha256(p) for p in output.iterdir() if p.is_file()}}
    (output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=64)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    result = render(args.capture, args.output, args.samples)
    print(json.dumps({"output": str(args.output), "metrics": result["metrics"]}, indent=2))
