"""Run inside Unreal's PythonScript commandlet with a verified native B05 capture.
Imports the original GLB with Interchange and captures stock materials to linear HDR.
See docs/97_UNREAL_B05_COMPARISON.md for project/command settings and limits.
"""
from __future__ import annotations

import argparse
from array import array
import json
import math
from pathlib import Path
import shutil
import struct
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cycles_b01 import B05_ID, compare_hdr, display_png, load_inputs, sha256, source_state, ROOT

B05_GLB = "d21ab7b6501c832fcc471be13d7d09586f2dc22932d7e9b89a9c103f985d5f2f"
EXPECTED_BOUNDS_CM = (((70, 40, 0), (130, 0, 130)),
                      ((0, -60, 0), (320, 0, 180)),
                      ((-70, 0, 0), (130, 0, 130)))


def to_unreal(vector, scale=1.0):
    if len(vector) != 3 or any(not math.isfinite(v) for v in vector):
        raise ValueError("finite three-component vector required")
    return [vector[0] * scale, vector[2] * scale, vector[1] * scale]


def camera_inputs(values):
    world = json.loads(values["capture_world_from_view"])
    projection = json.loads(values["capture_clip_from_view"])
    if (len(world) != 16 or len(projection) != 16
            or any(not math.isfinite(v) for v in (*world, *projection))
            or projection[0] <= 0 or projection[5] <= 0
            or projection[11] != -1 or projection[15] != 0):
        raise ValueError("captured finite perspective camera required")
    basis = [world[i:i+3] for i in (0,4,8)]
    if (world[15] != 1 or any(world[i] != 0 for i in (3,7,11))
            or any(not math.isclose(sum(v*v for v in axis),1,abs_tol=1e-6) for axis in basis)
            or any(abs(sum(a*b for a,b in zip(basis[i],basis[j])))>1e-6 for i,j in ((0,1),(0,2),(1,2)))
            or any(projection[i] != 0 for i in (1,2,3,4,6,7,8,9,12,13))):
        raise ValueError("unscaled orthonormal perspective camera required")
    return {"position_cm": to_unreal(world[12:15], 100),
            "forward": to_unreal([-v for v in world[8:11]]),
            "right": to_unreal(world[:3]), "up": to_unreal(world[4:7]),
            "horizontal_fov_degrees": math.degrees(2 * math.atan(1 / projection[0]))}


def match_bounds(origin, extent):
    for index, (center, half) in enumerate(EXPECTED_BOUNDS_CM):
        if all(math.isclose(a,b,abs_tol=0.001,rel_tol=0)
               for a,b in zip([*origin,*extent], [*center,*half])):
            return index
    raise ValueError(f"imported geometry bounds differ: {origin}, {extent}")


def source_emission(material):
    factor = material.get("emissiveFactor", [0,0,0])
    strength = material.get("extensions", {}).get("KHR_materials_emissive_strength", {}).get("emissiveStrength", 1)
    if (len(factor) != 3 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in factor)
            or not math.isfinite(strength) or strength < 0):
        raise ValueError("invalid source glTF emissive factor/strength")
    return {"factor": factor, "strength": strength, "radiance": [v*strength for v in factor]}


def render(directory: Path, output: Path):
    import unreal

    directory, output = directory.resolve(), output.resolve()
    if output == ROOT or ROOT in output.parents or output == directory or directory in output.parents:
        raise ValueError("use a new output directory outside the checkout/native capture")
    source = source_state(ROOT)
    adapter_hash = sha256(Path(__file__))
    capture, definition, settings = load_inputs(directory)
    if definition["id"] != B05_ID or capture["reference_geometry_sha256"] != B05_GLB:
        raise ValueError("only the frozen B05 v1 geometry is supported")
    input_hash = sha256(directory / "capture.json")
    camera = camera_inputs(capture["metadata"]["graphics"]["resolved"])
    glb = (directory / "scene.glb").read_bytes()
    document = json.loads(glb[20:20+struct.unpack_from("<I", glb, 12)[0]])
    output.mkdir(parents=True, exist_ok=False)
    for name in ("scene.glb", "scene.json", "scene.LICENSE.txt"):
        shutil.copyfile(directory / name, output / name)
    shutil.copyfile(directory / "display.png", output / "thessa.png")

    task = unreal.AssetImportTask()
    task.filename = str(output / "scene.glb")
    task.destination_path = "/Game/ReferenceB05"
    task.automated = True
    task.save = False
    task.replace_existing = True
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    imported = [unreal.load_asset(path) for path in task.imported_object_paths]
    meshes = [obj for obj in imported if isinstance(obj, unreal.StaticMesh)]
    if len(meshes) != 3:
        raise ValueError(f"expected three imported meshes, got {len(meshes)}")
    world = unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if any(isinstance(actor, (unreal.Light, unreal.SkyLight)) for actor in actors.get_all_level_actors()):
        raise ValueError("new reference world unexpectedly contains lighting")
    geometry = []
    for mesh in meshes:
        bounds = mesh.get_bounds()
        center = [bounds.origin.x, bounds.origin.y, bounds.origin.z]
        half = [bounds.box_extent.x, bounds.box_extent.y, bounds.box_extent.z]
        index = match_bounds(center, half)
        triangles = mesh.get_num_triangles(0)
        if triangles != 2 or len(mesh.static_materials) != 1:
            raise ValueError("B05 source triangle/material counts changed")
        actor = actors.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector())
        actor.static_mesh_component.set_static_mesh(mesh)
        material = mesh.static_materials[0].material_interface
        emission = source_emission(document["materials"][index])
        library = unreal.MaterialEditingLibrary
        if ("EmissiveFactor" not in [str(n) for n in library.get_vector_parameter_names(material)]
                or "EmissiveStrength" not in [str(n) for n in library.get_scalar_parameter_names(material)]):
            raise ValueError("stock Interchange emissive parameters unavailable")
        before = library.get_material_instance_vector_parameter_value(material, "EmissiveFactor")
        emission["imported_factor"] = [before.r,before.g,before.b]
        emission["imported_strength"] = library.get_material_instance_scalar_parameter_value(material, "EmissiveStrength")
        # Canonical literal glTF factor*strength, not a fitted image correction.
        # This Installed Build's texture-free stock graph drops the strength scalar.
        # Setters may return false for an unchanged value; verify by reading it back.
        library.set_material_instance_scalar_parameter_value(material, "EmissiveStrength", 1)
        library.set_material_instance_vector_parameter_value(material, "EmissiveFactor", unreal.LinearColor(*emission["radiance"],1))
        library.update_material_instance(material)
        after = library.get_material_instance_vector_parameter_value(material, "EmissiveFactor")
        emission["mapped_factor"] = [after.r,after.g,after.b]
        emission["mapped_strength"] = library.get_material_instance_scalar_parameter_value(material, "EmissiveStrength")
        if emission["mapped_strength"] != 1 or any(not math.isclose(a,b,abs_tol=1e-6) for a,b in zip(emission["mapped_factor"],emission["radiance"])):
            raise ValueError("literal emissive parameter mapping was not applied")
        # Public statistics waits for this material's shader map.
        unreal.MaterialEditingLibrary.get_statistics(material)
        geometry.append({"source_panel": index, "path": mesh.get_path_name(),
                         "triangles": triangles, "vertices": mesh.get_num_vertices(0),
                         "origin_cm": center, "extent_cm": half,
                         "material": material.get_path_name(),
                         "material_class": material.get_class().get_name(), "emission_mapping": emission})
    if sorted(g["source_panel"] for g in geometry) != [0,1,2]:
        raise ValueError("imported panel identity is not one-to-one")
    cvars = {"r.AntiAliasingMethod": 0, "r.EyeAdaptationQuality": 0,
             "r.BloomQuality": 0, "r.MotionBlurQuality": 0, "r.PreExposure": 0,
             "r.SeparateTranslucency": 0, "r.DynamicGlobalIlluminationMethod": 0,
             "r.ReflectionMethod": 0, "r.ScreenPercentage": 100, "r.SceneColorFormat": 4}
    for key, value in cvars.items():
        unreal.SystemLibrary.execute_console_command(world, f"{key} {value}")
        if unreal.SystemLibrary.get_console_variable_int_value(key) != value:
            raise ValueError(f"reference setting not applied: {key}")
    if unreal.SystemLibrary.get_console_variable_int_value("r.RayTracing") != 0:
        raise ValueError("explicit ray tracing off project required")
    actor = actors.spawn_actor_from_class(unreal.SceneCapture2D,
        unreal.Vector(*camera["position_cm"]), unreal.MathLibrary.make_rotation_from_axes(
            unreal.Vector(*camera["forward"]), unreal.Vector(*camera["right"]), unreal.Vector(*camera["up"])))
    component = actor.capture_component2d
    component.capture_every_frame = False
    component.capture_on_movement = False
    component.capture_source = unreal.SceneCaptureSource.SCS_SCENE_COLOR_HDR
    component.fov_angle = camera["horizontal_fov_degrees"]
    component.override_custom_near_clipping_plane = True
    component.custom_near_clipping_plane = definition["near_m"] * 100
    component.max_view_distance_override = definition["far_m"] * 100
    vector = lambda v: [v.x,v.y,v.z]
    actual_camera = {"position_cm": vector(actor.get_actor_location()),
                     "forward": vector(actor.get_actor_forward_vector()),
                     "right": vector(actor.get_actor_right_vector()), "up": vector(actor.get_actor_up_vector()),
                     "horizontal_fov_degrees": component.fov_angle,
                     "near_cm": component.custom_near_clipping_plane,
                     "max_draw_distance_cm": component.max_view_distance_override}
    width, height = settings["width"], settings["height"]
    target = unreal.RenderingLibrary.create_render_target2d(world, width, height,
        unreal.TextureRenderTargetFormat.RTF_RGBA16F, unreal.LinearColor(0,0,0,1), False, False)
    component.texture_target = target
    for _ in range(3):
        component.capture_scene()
    pixels = unreal.RenderingLibrary.read_render_target_raw(world, target, False)
    if pixels is None or len(pixels) != width * height:
        raise ValueError("Unreal raw HDR readback extent differs")
    raw, exposed = array("f"), array("f")
    for pixel in pixels:
        sample = (pixel.r,pixel.g,pixel.b,pixel.a)
        if any(not math.isfinite(v) for v in sample):
            raise ValueError("non-finite Unreal HDR readback")
        raw.extend(sample)
        exposed.extend((*(v * settings["exposure_gain"] for v in sample[:3]), sample[3]))
    if not any(v > 0 for i,v in enumerate(raw) if i % 4 != 3):
        raise ValueError("Unreal capture contains no emissive signal")
    for name, values in (("radiance.rgba32f", raw), ("linear.rgba32f", exposed)):
        data = array("f", values)
        if sys.byteorder != "little": data.byteswap()
        (output / name).write_bytes(data.tobytes())
    display_png(output / "display.png", exposed, width, height)
    if (source_state(ROOT) != source or sha256(Path(__file__)) != adapter_hash
            or sha256(directory / "capture.json") != input_hash or load_inputs(directory)[0] != capture
            or sha256(output / "scene.glb") != B05_GLB):
        raise ValueError("source/native inputs changed during Unreal capture")
    report = {"schema_version": 1, "scene_id": B05_ID, "adapter_source": source,
              "adapter_sha256": adapter_hash, "native_capture_sha256": input_hash,
              "reference_geometry_sha256": B05_GLB, "native_source": capture["source"],
              "engine": {"name": "Unreal Engine", "version": unreal.SystemLibrary.get_engine_version()},
              "capture": {**settings, "camera_requested": camera, "camera_actual": actual_camera,
                          "cvars": cvars, "ray_tracing_verified_off": True, "empty_light_world_verified": True,
                          "warmup_captures": 3,
                          "format": "RGBA16Float read as float32; normalize=False",
                          "source": "SCS_SCENE_COLOR_HDR; scene-linear RGB, inverse opacity alpha",
                          "alpha_comparison": "RGB only; native and capture alpha have different contracts",
                          "tone_mapping": "shared native gain, per-channel Reinhard, sRGB; Unreal tonemapper excluded",
                          "coordinate_conversion": "glTF metres (x,y,z) -> Unreal cm 100*(x,z,y)",
                          "geometry": geometry,
                          "material_mapping": "Stock Interchange geometry/PBR/alpha; literal glTF factor*strength baked into existing EmissiveFactor, EmissiveStrength=1; no shader graph/BRDF override"},
              "metrics": compare_hdr(directory / "linear.rgba16f", exposed),
              "acceptance": {"physical": "not evaluated", "visual": "not evaluated",
                             "performance": "not measured/comparable", "temporal": "not evaluated"},
              "limits": ["Frozen emission-only B05; not glass/refraction/GI or coastal scene parity",
                         "No absolute photometric calibration; no incident-light unit mapping needed",
                         "Different engine rasterization/import/shader precision; silhouette differences possible",
                         "Readback/cold shader compilation duration is not frame time"],
              "artifacts_sha256": {p.name: sha256(p) for p in output.iterdir() if p.is_file()}}
    (output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    render(args.capture, args.output)
