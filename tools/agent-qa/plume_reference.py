"""Frozen analytic plume-volume controls, not a calibrated engine or flight replay.

capture: build/run the actual native GPU fixture with source/binary identities.
cycles: run inside Blender over that immutable publication; stock Volume Coefficients.
See docs/100_PLUME_SEGMENT_TRANSPORT.md for contracts, commands and limits.
"""
from __future__ import annotations

import argparse
from array import array
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cycles_b01 import ROOT, compare_hdr, display_png, sha256, source_state

CASES = ("axial-zero", "axial-one", "axial-ten", "side")
ID = "analytic-plume-volume-v1"


def validate_scene(scene):
    if (scene["id"] != ID or scene["case"] not in CASES or scene["extent"] != [129,129]
            or scene["units"] != "metres" or scene["exposure_gain"] != 1
            or scene["steps"] != 14 or scene["erosion"] != 0 or scene["tail_dissolve"]
            or not scene["diamonds"]):
        raise ValueError("only frozen, unanimated analytic volume controls are supported")
    origin, axis, profile = scene["origin_len"], scene["axis_radius"], scene["profile"]
    world, projection = scene["world_from_view"], scene["clip_from_view"]
    if (len(origin) != 4 or len(axis) != 4 or len(profile) != 2
            or any(len(p) != 8 for p in profile) or len(world) != 16 or len(projection) != 16
            or any(not math.isfinite(v) for v in (*origin,*axis,*profile[0],*profile[1],*world,*projection))
            or origin[3] <= 0 or axis[3] <= 0 or not math.isclose(sum(v*v for v in axis[:3]),1)
            or world != [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]
            or projection[0] <= 0 or projection[5] <= 0 or projection[11] != -1
            or projection[15] != 0 or any(projection[i] != 0 for i in (1,2,3,4,6,7,8,9,12,13))
            or profile[0][0] != 0 or profile[1][0] != origin[3]
            or profile[0][1:] != profile[1][1:] or profile[0][1] <= 0
            or axis[3] > 3*profile[0][1]
            or profile[0][2] < 0 or profile[0][3] != 1 or min(profile[0][4:7]) < 0):
        raise ValueError("finite, constant axial publication and fixed perspective camera required")
    return scene


def bounds(scene):
    origin, axis = scene["origin_len"], scene["axis_radius"]
    center = [origin[i] + axis[i]*origin[3]/2 for i in range(3)]
    half = [abs(axis[i])*origin[3]/2 + axis[3]*math.sqrt(max(0,1-axis[i]**2)) for i in range(3)]
    return center, half


def analytic_pixels(scene):
    """Exact Gaussian line integral for this constant axial field, at pixel centres.

    This does not reuse the CPU/GPU marcher. With constant emission/extinction
    ratio the equation of transfer reduces to j/k * (1-exp(-k*integral(w ds))).
    The radial Gaussian mixture integrates with erf, including finite cylinder caps.
    """
    validate_scene(scene)
    origin, axis, station = scene["origin_len"], scene["axis_radius"][:3], scene["profile"][0]
    oc = [-v for v in origin[:3]]
    dot = lambda a,b: sum(x*y for x,y in zip(a,b))
    oa = dot(oc,axis); lateral_origin = [oc[i]-axis[i]*oa for i in range(3)]
    q = dot(lateral_origin,lateral_origin); result = []
    width,height = scene["extent"]; projection = scene["clip_from_view"]
    for y in range(height):
        for x in range(width):
            rd = [(2*(x+0.5)/width-1)/projection[0],(1-2*(y+0.5)/height)/projection[5],-1]
            norm = math.sqrt(dot(rd,rd)); rd = [v/norm for v in rd]
            da = dot(rd,axis); lateral_ray = [rd[i]-axis[i]*da for i in range(3)]
            a,b,c = dot(lateral_ray,lateral_ray),dot(lateral_origin,lateral_ray),q-scene["axis_radius"][3]**2
            near,far = 0.0,math.inf
            if a == 0:
                if c > 0: result.extend([0]*4);continue
            else:
                disc=b*b-a*c
                if disc <= 0: result.extend([0]*4);continue
                root=math.sqrt(disc);near=max(near,(-b-root)/a);far=min(far,(-b+root)/a)
            if da == 0:
                if not 0 <= oa <= origin[3]:result.extend([0]*4);continue
            else:
                caps=sorted([-oa/da,(origin[3]-oa)/da]);near=max(near,caps[0]);far=min(far,caps[1])
            if far <= near:result.extend([0]*4);continue
            def gaussian(rate):
                rate /= station[1]**2
                if a == 0:return (far-near)*math.exp(-rate*q)
                shift=-b/a;offset=max(0,q-b*b/a);scale=math.sqrt(rate*a)
                return math.exp(-rate*offset)*math.sqrt(math.pi)/(2*scale)*(math.erf(scale*(far-shift))-math.erf(scale*(near-shift)))
            integral=gaussian(2.2)+0.18*gaussian(0.7)
            alpha=-math.expm1(-station[2]*integral)
            length=alpha/station[2] if station[2] > 0 else integral
            result.extend([v*length for v in station[4:7]]+[alpha])
    return result


def new_output(path, parent=None):
    path = path.resolve()
    if path == ROOT or ROOT in path.parents or (parent and (path == parent or parent in path.parents)):
        raise ValueError("new output must be outside the checkout and native capture")
    path.mkdir(parents=True, exist_ok=False)
    return path


def native_pixels(directory):
    raw = (directory/"linear.rgba16f").read_bytes()
    if len(raw) != 129*129*8: raise ValueError("native HDR extent differs")
    values = [x[0] for x in struct.iter_unpack("<e",raw)]
    if any(not math.isfinite(v) or v < 0 for v in values) or not any(v > 0 for i,v in enumerate(values) if i%4 != 3):
        raise ValueError("non-finite, negative or empty native HDR")
    return values


def capture(output, profile, allow_dirty):
    source = source_state(ROOT)
    if source["dirty"] and not allow_dirty:
        raise ValueError("commit first or explicitly record --allow-dirty")
    output = new_output(output)
    command = ["cargo","test","--offline","-p","thessa-render-wgpu","--test","plume","--no-run","--message-format=json"]
    if profile == "release": command.append("--release")
    with (output/"build.stderr.log").open("w") as log:
        built = subprocess.run(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=log,text=True,check=True,timeout=600)
    (output/"build.jsonl").write_text(built.stdout)
    artifacts = [json.loads(line) for line in built.stdout.splitlines()]
    executables = [m["executable"] for m in artifacts if m.get("reason")=="compiler-artifact"
                   and m.get("target",{}).get("name")=="plume" and m.get("executable")]
    if len(executables) != 1: raise ValueError("one native plume test executable required")
    binary = Path(executables[0]); binary_hash = sha256(binary)
    command = [str(binary),"frozen_plume_volume_reference","--ignored","--nocapture","--test-threads","1"]
    import os
    env = {**os.environ,"THESSA_BENCH_OUTPUT":str(output/"native")}
    with (output/"native.log").open("w") as log:
        subprocess.run(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
    identities = {}
    for case in CASES:
        directory = output/"native"/case
        scene = validate_scene(json.loads((directory/"scene.json").read_text()))
        values = native_pixels(directory)
        display_png(directory/"display.png",values,*scene["extent"])
        exact = analytic_pixels(scene)
        display_png(directory/"analytic.png",exact,*scene["extent"])
        metrics = compare_hdr(directory/"linear.rgba16f",exact)
        metrics["interpretation"] = "Native midpoint integration/half-float versus exact Gaussian line integral at pixel centres; no reconstruction filter or engine calibration"
        alpha_error = [abs(values[i]-exact[i]) for i in range(3,len(values),4)]
        (directory/"analytic.json").write_text(json.dumps({"method":"Gaussian erf line integral; pixel centres, no reconstruction filter",
            "metrics":metrics,"max_absolute_alpha_error":max(alpha_error),
            "center_linear_rgba":exact[(64*129+64)*4:(64*129+64)*4+4]},indent=2)+"\n")
        identities[case] = {p.name:sha256(p) for p in directory.iterdir() if p.is_file()}
    if source_state(ROOT) != source or sha256(binary) != binary_hash:
        raise ValueError("native source/binary changed during capture")
    record = {"id":ID,"source":source,"binary_sha256":binary_hash,"profile":profile,
              "tool_sha256":sha256(Path(__file__)),"native_command":command,"cases":identities,
              "scope":"analytic field controls; not a physical engine, full-frame benchmark or preference acceptance"}
    (output/"capture.json").write_text(json.dumps(record,indent=2)+"\n")
    return record


def load_inputs(directory):
    record = json.loads((directory/"capture.json").read_text())
    if record["id"] != ID or set(record["cases"]) != set(CASES): raise ValueError("volume control identity differs")
    for case in CASES:
        files = record["cases"][case]
        if set(files) != {"scene.json","linear.rgba16f","display.png","analytic.png","analytic.json"}: raise ValueError("unexpected volume artifacts")
        for name,digest in files.items():
            if sha256(directory/"native"/case/name) != digest: raise ValueError("native volume artifact changed")
        validate_scene(json.loads((directory/"native"/case/"scene.json").read_text()))
        native_pixels(directory/"native"/case)
    return record


def cycles(directory, output, samples):
    import bpy
    from mathutils import Matrix

    if not 1 <= samples <= 2048: raise ValueError("samples must be in 1..2048")
    directory = directory.resolve(); record = load_inputs(directory)
    identity = sha256(directory/"capture.json"); source = source_state(ROOT)
    output = new_output(output,directory); reports = {}
    for case in CASES:
        original = directory/"native"/case
        field = validate_scene(json.loads((original/"scene.json").read_text()))
        destination = output/case; destination.mkdir()
        bpy.ops.wm.read_factory_settings(use_empty=True)
        scene = bpy.context.scene; scene.render.engine = "CYCLES"; scene.cycles.device = "CPU"
        scene.cycles.samples = samples; scene.cycles.seed = 0; scene.cycles.use_animated_seed = False
        scene.cycles.use_denoising = False; scene.cycles.use_adaptive_sampling = False
        scene.cycles.max_bounces = 2; scene.cycles.volume_bounces = 0
        scene.cycles.volume_step_rate = 0.1; scene.cycles.volume_max_steps = 4096
        scene.cycles.volume_biased = False
        scene.render.resolution_x,scene.render.resolution_y = field["extent"]
        scene.render.resolution_percentage = 100; scene.render.pixel_aspect_x = scene.render.pixel_aspect_y = 1
        scene.render.use_compositing = False; scene.render.use_sequencer = False
        scene.render.film_transparent = False; scene.render.dither_intensity = 0
        scene.view_settings.view_transform = "Standard"; scene.view_settings.look = "None"
        scene.view_settings.exposure = 0; scene.view_settings.gamma = 1
        scene.world = bpy.data.worlds.new("black environment"); scene.world.use_nodes = True
        scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0
        center, half = bounds(field)
        verts = [tuple(center[i]+sign[i]*half[i] for i in range(3))
                 for sign in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        mesh = bpy.data.meshes.new("volume carrier"); mesh.from_pydata(verts,[],[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
        obj = bpy.data.objects.new("analytic cylinder in carrier box",mesh); scene.collection.objects.link(obj)
        material = bpy.data.materials.new("source coefficients, no scattering"); material.use_nodes = True
        nodes,links = material.node_tree.nodes,material.node_tree.links; nodes.clear()
        def assign(socket, value):
            if isinstance(value,(int,float,list,tuple)): socket.default_value = value
            else: links.new(value,socket)
        def scalar(op,a,b=0):
            n=nodes.new("ShaderNodeMath"); n.operation=op; assign(n.inputs[0],a); assign(n.inputs[1],b); return n.outputs[0]
        def vector(op,a,b):
            n=nodes.new("ShaderNodeVectorMath"); n.operation=op; assign(n.inputs[0],a)
            assign(n.inputs["Scale"] if op=="SCALE" else n.inputs[1],b)
            return n.outputs["Value"] if op=="DOT_PRODUCT" else n.outputs["Vector"]
        position = nodes.new("ShaderNodeNewGeometry").outputs["Position"]
        p = vector("SUBTRACT",position,field["origin_len"][:3]); axis=field["axis_radius"][:3]
        z = vector("DOT_PRODUCT",p,axis); radial = vector("SUBTRACT",p,vector("SCALE",axis,z))
        radius_squared = vector("DOT_PRODUCT",radial,radial); station = field["profile"][0]
        u2=scalar("MINIMUM",scalar("DIVIDE",radius_squared,station[1]**2),9)
        weight=scalar("ADD",scalar("EXPONENT",scalar("MULTIPLY",u2,-2.2)),scalar("MULTIPLY",scalar("EXPONENT",scalar("MULTIPLY",u2,-0.7)),0.18))
        support=scalar("MULTIPLY",scalar("GREATER_THAN",z,0),scalar("LESS_THAN",z,field["origin_len"][3]))
        support=scalar("MULTIPLY",support,scalar("LESS_THAN",radius_squared,field["axis_radius"][3]**2))
        weight=scalar("MULTIPLY",weight,support)
        volume=nodes.new("ShaderNodeVolumeCoefficients")
        volume.inputs["Scatter Coefficients"].default_value=(0,0,0)
        links.new(vector("SCALE",[station[2]]*3,weight),volume.inputs["Absorption Coefficients"])
        links.new(vector("SCALE",station[4:7],weight),volume.inputs["Emission Coefficients"])
        links.new(volume.outputs["Volume"],nodes.new("ShaderNodeOutputMaterial").inputs["Volume"])
        mesh.materials.append(material)
        camera=bpy.data.objects.new("captured camera",bpy.data.cameras.new("captured camera"));scene.collection.objects.link(camera)
        m=field["world_from_view"];camera.matrix_world=Matrix(tuple(tuple(m[c*4+r] for c in range(4)) for r in range(4)))
        camera.data.sensor_fit="VERTICAL";camera.data.sensor_height=24;camera.data.lens=12*field["clip_from_view"][5]
        camera.data.clip_start=0.1;camera.data.clip_end=1e6;scene.camera=camera
        scene.render.image_settings.file_format="OPEN_EXR";scene.render.image_settings.color_mode="RGBA"
        scene.render.image_settings.color_depth="32";scene.render.image_settings.exr_codec="ZIP"
        scene.render.filepath=str(destination/"radiance.exr");bpy.ops.render.render(write_still=True)
        image=bpy.data.images.load(scene.render.filepath,check_existing=False)
        if list(image.size) != field["extent"]: raise ValueError("Cycles volume extent differs")
        pixels=array("f",[0])*(129*129*4);image.pixels.foreach_get(pixels)
        rgba=array("f")
        for y in reversed(range(129)): rgba.extend(pixels[y*129*4:(y+1)*129*4])
        if any(not math.isfinite(v) for v in rgba): raise ValueError("non-finite Cycles volume")
        binary=array("f",rgba)
        if sys.byteorder!="little":binary.byteswap()
        (destination/"linear.rgba32f").write_bytes(binary.tobytes());display_png(destination/"display.png",rgba,129,129)
        shutil.copyfile(original/"display.png",destination/"thessa.png");shutil.copyfile(original/"scene.json",destination/"scene.json")
        bpy.ops.wm.save_as_mainfile(filepath=str(destination/"scene.blend"))
        bpy.context.view_layer.update();mesh.calc_loop_triangles()
        actual_camera={"world_from_view_rows":[list(row) for row in camera.matrix_world],
                       "vertical_fov_rad":camera.data.angle_y,"near_m":camera.data.clip_start,"far_m":camera.data.clip_end}
        expected_fov=2*math.atan(1/field["clip_from_view"][5])
        if (not math.isclose(actual_camera["vertical_fov_rad"],expected_fov,abs_tol=1e-6)
                or len(mesh.loop_triangles)!=12 or obj.matrix_world!=Matrix.Identity(4)):
            raise ValueError("actual Cycles camera or unscaled carrier differs")
        metrics=compare_hdr(original/"linear.rgba16f",rgba)
        metrics["interpretation"]="Native raster/midpoint/half-float versus Cycles stochastic volume transport and pixel reconstruction; diagnostic only"
        center_index=(64*129+64)*4
        reports[case]={"metrics":metrics,"center_linear_rgba":list(rgba[center_index:center_index+4]),
                       "actual_camera":actual_camera,"carrier_triangles":len(mesh.loop_triangles),
                       "material_node":"ShaderNodeVolumeCoefficients","scatter_coefficients":list(volume.inputs["Scatter Coefficients"].default_value),
                       "artifacts_sha256":{p.name:sha256(p) for p in destination.iterdir() if p.is_file()}}
    if source_state(ROOT)!=source or sha256(directory/"capture.json")!=identity or load_inputs(directory)!=record:
        raise ValueError("adapter source or native inputs changed during comparison")
    report={"id":ID,"engine":{"name":"Blender/Cycles","version":bpy.app.version_string,"build_hash":bpy.app.build_hash.decode()},
            "source":source,"tool_sha256":sha256(Path(__file__)),"native_capture_sha256":identity,"native_source":record["source"],
            "settings":{"device":"CPU","samples":samples,"seed":0,"volume_step_rate":0.1,"volume_max_steps":4096,"volume_biased":False,"scattering":0,
                        "alpha_comparison":"RGB only; Cycles opaque film alpha is not transmittance","display":"shared Reinhard/sRGB, gain=1; native raster vs Cycles pixel reconstruction"},
            "cases":reports,"acceptance":{"physical":"analytic transport control only; not engine calibration","visual":"not evaluated","performance":"not comparable","temporal":"not evaluated"}}
    (output/"comparison.json").write_text(json.dumps(report,indent=2)+"\n")
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__); commands=parser.add_subparsers(dest="command",required=True)
    native=commands.add_parser("capture");native.add_argument("--output",type=Path,required=True)
    native.add_argument("--profile",choices=("dev","release"),default="dev");native.add_argument("--allow-dirty",action="store_true")
    external=commands.add_parser("cycles");external.add_argument("capture",type=Path);external.add_argument("--output",type=Path,required=True)
    external.add_argument("--samples",type=int,default=256)
    args=parser.parse_args(sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else None)
    if args.command=="capture":capture(args.output,args.profile,args.allow_dirty)
    else:cycles(args.capture,args.output,args.samples)
