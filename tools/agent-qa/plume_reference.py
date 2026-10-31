"""Frozen analytic and compiled-engine volume controls, not calibrated firings or flight replay.

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
ENGINE_ID = "compiled-plume-volume-v1"
MOTION_ID = "curved-plume-volume-v1"
MOTION_CASES = ("sea-full", "thin-full", "matched-inside")
ENGINE_CASES = ("sea-full", "sea-half", "matched-full", "matched-half", "thin-full", "thin-half",
                "matched-axial", "matched-inside", "matched-oblique")


def cases_for(identity):
    if identity == ID: return CASES
    if identity == ENGINE_ID: return ENGINE_CASES
    if identity == MOTION_ID: return MOTION_CASES
    raise ValueError("unsupported plume publication")


def validate_scene(scene):
    identity = scene["id"]; cases = cases_for(identity)
    if scene.get("radial_weight_mode", "clamped-gaussian-v1") not in ("clamped-gaussian-v1", "gaussian-v2"):
        raise ValueError("unknown radial density model")
    if scene.get("curve_metric", "unscaled-v1") not in ("unscaled-v1", "emitted-length-v1"):
        raise ValueError("unknown curve metric")
    if scene.get("shock_radial_mode", "uniform-v1") not in ("uniform-v1", "axisymmetric-j0-v1"):
        raise ValueError("unknown radial shock model")
    if (scene["case"] not in cases or scene["extent"] != [129,129]
            or scene["units"] != "metres" or scene["exposure_gain"] != 1
            or scene["steps"] != 14 or scene["erosion"] != 0 or scene["tail_dissolve"]
            or not scene["diamonds"]):
        raise ValueError("only frozen, unanimated volume controls are supported")
    origin, axis, profile = scene["origin_len"], scene["axis_radius"], scene["profile"]
    world, projection = scene["world_from_view"], scene["clip_from_view"]
    if (len(origin) != 4 or len(axis) != 4 or len(profile) != (2 if identity == ID else 48)
            or any(len(p) != 8 for p in profile) or len(world) != 16 or len(projection) != 16
            or any(not math.isfinite(v) for v in (*origin,*axis,*(v for p in profile for v in p),*world,*projection))
            or origin[3] <= 0 or axis[3] <= 0 or not math.isclose(sum(v*v for v in axis[:3]),1,abs_tol=1e-6)
            or world != [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]
            or projection[0] <= 0 or projection[5] <= 0 or projection[11] != -1
            or projection[15] != 0 or any(projection[i] != 0 for i in (1,2,3,4,6,7,8,9,12,13))
            or profile[0][0] != 0 or profile[-1][0] != origin[3]
            or any(p[1] <= 0 or p[2] < 0 or p[3] <= 0 or min(p[4:7]) < 0 for p in profile)
            or any(a[0] >= b[0] for a,b in zip(profile,profile[1:]))):
        raise ValueError("finite ordered profile and fixed perspective camera required")
    if identity == ID:
        if (profile[0][1:] != profile[1][1:] or profile[0][3] != 1 or axis[3] > 3*profile[0][1]):
            raise ValueError("constant axial analytic control required")
    else:
        if (scene["source_throttle"] != 1 or scene["effective_throttle"] not in (0.5,1)
                or not all(math.isfinite(scene[k]) and scene[k] > 0 for k in
                           ("exit_radius_m","ambient_pressure_pa","ambient_temperature_k"))
                or not all(math.isfinite(v) for v in scene["operating_point"].values())
                or scene["operating_point"]["mass_flow_kg_s"] <= 0
                or not math.isclose(axis[3],1.6*max(p[1] for p in profile),rel_tol=2e-7)
                or any(abs(p[0]-origin[3]*i/47) > origin[3]*2e-7 for i,p in enumerate(profile))):
            raise ValueError("compiled operating source and uniformly spaced f32 stations required")
    centers = scene.get("centerline", [])
    if identity == MOTION_ID:
        if (not 2 <= len(centers) <= 33 or any(len(p) != 4 for p in centers)
                or any(not math.isfinite(v) for p in centers for v in p)
                or centers[0][3] != 0 or centers[-1][3] != origin[3]
                or any(a[3] >= b[3] or sum((a[i]-b[i])**2 for i in range(3)) <= 1e-12
                       for a,b in zip(centers,centers[1:]))):
            raise ValueError("finite ordered swept centerline required")
    elif centers:
        raise ValueError("straight control must not contain a swept centerline")
    return scene


def bounds(scene):
    origin, axis = scene["origin_len"], scene["axis_radius"]
    if scene.get("centerline"):
        lo = [min(p[i] for p in scene["centerline"])-axis[3] for i in range(3)]
        hi = [max(p[i] for p in scene["centerline"])+axis[3] for i in range(3)]
        return [(a+b)/2 for a,b in zip(lo,hi)], [(b-a)/2 for a,b in zip(lo,hi)]
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
    if scene["id"] != ID: raise ValueError("Gaussian closed form requires a constant axial control")
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


def native_pixels(directory, filename="linear.rgba16f"):
    raw = (directory/filename).read_bytes()
    if len(raw) != 129*129*8: raise ValueError("native HDR extent differs")
    values = [x[0] for x in struct.iter_unpack("<e",raw)]
    if any(not math.isfinite(v) or v < 0 for v in values) or not any(v > 0 for i,v in enumerate(values) if i%4 != 3):
        raise ValueError("non-finite, negative or empty native HDR")
    return values


def capture(output, profile, allow_dirty, suite="analytic"):
    identity = {"analytic":ID, "compiled":ENGINE_ID, "motion":MOTION_ID}[suite]
    cases = cases_for(identity)
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
    command = [str(binary),{ID:"frozen_plume_volume_reference",ENGINE_ID:"frozen_compiled_plume_reference",MOTION_ID:"frozen_curved_plume_reference"}[identity],"--ignored","--nocapture","--test-threads","1"]
    import os
    env = {**os.environ,"THESSA_BENCH_OUTPUT":str(output/"native")}
    with (output/"native.log").open("w") as log:
        subprocess.run(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
    identities = {}
    for case in cases:
        directory = output/"native"/case
        scene = validate_scene(json.loads((directory/"scene.json").read_text()))
        values = native_pixels(directory)
        display_png(directory/"display.png",values,*scene["extent"])
        if identity == ID:
            exact = analytic_pixels(scene)
            display_png(directory/"analytic.png",exact,*scene["extent"])
            metrics = compare_hdr(directory/"linear.rgba16f",exact)
            metrics["interpretation"] = "Native midpoint integration/half-float versus exact Gaussian line integral at pixel centres; no reconstruction filter or engine calibration"
            alpha_error = [abs(values[i]-exact[i]) for i in range(3,len(values),4)]
            (directory/"analytic.json").write_text(json.dumps({"method":"Gaussian erf line integral; pixel centres, no reconstruction filter",
                "metrics":metrics,"max_absolute_alpha_error":max(alpha_error),
                "center_linear_rgba":exact[(64*129+64)*4:(64*129+64)*4+4]},indent=2)+"\n")
        else:
            sixteen = native_pixels(directory, "steps16.rgba16f")
            display_png(directory/"steps16.png",sixteen,*scene["extent"])
        identities[case] = {p.name:sha256(p) for p in directory.iterdir() if p.is_file()}
    if source_state(ROOT) != source or sha256(binary) != binary_hash:
        raise ValueError("native source/binary changed during capture")
    record = {"id":identity,"source":source,"binary_sha256":binary_hash,"profile":profile,
              "tool_sha256":sha256(Path(__file__)),"native_command":command,"cases":identities,
              "scope":"frozen transport controls; compiled cases use model operating points, not calibrated firings, full-frame performance or preference acceptance"}
    (output/"capture.json").write_text(json.dumps(record,indent=2)+"\n")
    return record


def load_inputs(directory):
    record = json.loads((directory/"capture.json").read_text())
    cases = cases_for(record["id"])
    if set(record["cases"]) != set(cases): raise ValueError("volume control identity differs")
    for case in cases:
        files = record["cases"][case]
        expected = {"scene.json","linear.rgba16f","display.png"} | ({"analytic.png","analytic.json"} if record["id"] == ID else {"steps16.rgba16f","steps16.png"})
        if set(files) != expected: raise ValueError("unexpected volume artifacts")
        for name,digest in files.items():
            if sha256(directory/"native"/case/name) != digest: raise ValueError("native volume artifact changed")
        scene = validate_scene(json.loads((directory/"native"/case/"scene.json").read_text()))
        if scene["id"] != record["id"] or scene["case"] != case: raise ValueError("case identity differs")
        native_pixels(directory/"native"/case)
        if record["id"] in (ENGINE_ID, MOTION_ID): native_pixels(directory/"native"/case, "steps16.rgba16f")
    return record


def cycles(directory, output, samples):
    import bpy
    from mathutils import Matrix

    if not 1 <= samples <= 2048: raise ValueError("samples must be in 1..2048")
    directory = directory.resolve(); record = load_inputs(directory)
    identity = sha256(directory/"capture.json"); source = source_state(ROOT)
    output = new_output(output,directory); reports = {}
    for case in cases_for(record["id"]):
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
        radius_squared = vector("DOT_PRODUCT",radial,radial)
        metric = 1
        if field.get("centerline"):
            # Stock Cycles material nodes implement the declared swept field;
            # the renderer/integrator is unchanged. Equal-distance ties keep
            # the first segment, just like the native field definition.
            radius_squared = 1e30; z = 0
            centers = field["centerline"]
            def select(a,b,t):
                return scalar("ADD",scalar("MULTIPLY",a,scalar("SUBTRACT",1,t)),scalar("MULTIPLY",b,t))
            for j,(a,b) in enumerate(zip(centers,centers[1:])):
                ab=[b[i]-a[i] for i in range(3)]
                t=scalar("DIVIDE",vector("DOT_PRODUCT",vector("SUBTRACT",position,a[:3]),ab),sum(v*v for v in ab))
                valid=1
                if j == 0: valid=scalar("SUBTRACT",1,scalar("LESS_THAN",t,0))
                if j == len(centers)-2: valid=scalar("MULTIPLY",valid,scalar("SUBTRACT",1,scalar("GREATER_THAN",t,1)))
                fraction=scalar("MINIMUM",scalar("MAXIMUM",t,0),1)
                delta=vector("SUBTRACT",position,vector("ADD",a[:3],vector("SCALE",ab,fraction)))
                d2=vector("DOT_PRODUCT",delta,delta)
                wins=scalar("MULTIPLY",valid,scalar("LESS_THAN",d2,radius_squared))
                z=select(z,scalar("ADD",a[3],scalar("MULTIPLY",b[3]-a[3],fraction)),wins)
                if field.get("curve_metric", "unscaled-v1") == "emitted-length-v1":
                    metric=select(metric,(b[3]-a[3])/math.sqrt(sum(v*v for v in ab)),wins)
                radius_squared=select(radius_squared,d2,wins)
        station = field["profile"][0]
        if field["id"] == ID:
            radius, extinction, shock, emission = station[1], station[2], station[3], station[4:7]
        else:
            # Float textures are only a stock node-graph representation of the
            # frozen uniform axial samples. No baked color/shape or fitted gain.
            count = len(field["profile"])
            u = scalar("DIVIDE",scalar("ADD",scalar("MULTIPLY",scalar("DIVIDE",z,field["origin_len"][3]),count-1),0.5),count)
            uv = nodes.new("ShaderNodeCombineXYZ"); assign(uv.inputs["X"],u); uv.inputs["Y"].default_value=0.5
            def table(name, values):
                image=bpy.data.images.new(name,width=count,height=1,alpha=True,float_buffer=True)
                image.colorspace_settings.name="Non-Color"; image.alpha_mode="CHANNEL_PACKED"
                image.pixels.foreach_set([v for row in values for v in row]); image.pack()
                n=nodes.new("ShaderNodeTexImage");n.image=image;n.interpolation="Linear";n.extension="EXTEND"
                links.new(uv.outputs["Vector"],n.inputs["Vector"]);return n.outputs["Color"]
            geometry=table("captured radius extinction shock",[p[1:4]+[1] for p in field["profile"]])
            separate=nodes.new("ShaderNodeSeparateColor");links.new(geometry,separate.inputs["Color"])
            radius,extinction,shock=separate.outputs["Red"],separate.outputs["Green"],separate.outputs["Blue"]
            emission=table("captured linear emission",[p[4:7]+[1] for p in field["profile"]])
        u2=scalar("DIVIDE",radius_squared,scalar("MULTIPLY",radius,radius))
        if field.get("radial_weight_mode", "clamped-gaussian-v1") == "clamped-gaussian-v1":
            u2=scalar("MINIMUM",u2,9)
        weight=scalar("ADD",scalar("EXPONENT",scalar("MULTIPLY",u2,-2.2)),scalar("MULTIPLY",scalar("EXPONENT",scalar("MULTIPLY",u2,-0.7)),0.18))
        support=scalar("MULTIPLY",scalar("GREATER_THAN",z,0),scalar("LESS_THAN",z,field["origin_len"][3]))
        support=scalar("MULTIPLY",support,scalar("LESS_THAN",radius_squared,field["axis_radius"][3]**2))
        shock=scalar("MAXIMUM",shock,0.15)
        if field.get("shock_radial_mode", "uniform-v1") == "axisymmetric-j0-v1":
            q=scalar("MULTIPLY",scalar("MINIMUM",u2,1),1.4457964907366962)
            mode=1/518400
            for coefficient in (-1/14400,1/576,-1/36,0.25,-1,1):
                mode=scalar("ADD",coefficient,scalar("MULTIPLY",q,mode))
            mode=scalar("MULTIPLY",scalar("MINIMUM",scalar("MAXIMUM",mode,0),1),scalar("LESS_THAN",u2,1))
            shock=scalar("MAXIMUM",scalar("ADD",1,scalar("MULTIPLY",scalar("SUBTRACT",shock,1),mode)),0.15)
        weight=scalar("MULTIPLY",scalar("MULTIPLY",scalar("MULTIPLY",weight,support),shock),metric)
        volume=nodes.new("ShaderNodeVolumeCoefficients")
        volume.inputs["Scatter Coefficients"].default_value=(0,0,0)
        absorb=scalar("MULTIPLY",extinction,weight)
        absorb_rgb=nodes.new("ShaderNodeCombineXYZ")
        for component in ("X","Y","Z"):assign(absorb_rgb.inputs[component],absorb)
        links.new(absorb_rgb.outputs["Vector"],volume.inputs["Absorption Coefficients"])
        links.new(vector("SCALE",emission,weight),volume.inputs["Emission Coefficients"])
        links.new(volume.outputs["Volume"],nodes.new("ShaderNodeOutputMaterial").inputs["Volume"])
        mesh.materials.append(material)
        camera=bpy.data.objects.new("captured camera",bpy.data.cameras.new("captured camera"));scene.collection.objects.link(camera)
        m=field["world_from_view"];camera.matrix_world=Matrix(tuple(tuple(m[c*4+r] for c in range(4)) for r in range(4)))
        camera.data.sensor_fit="VERTICAL";camera.data.sensor_height=24;camera.data.lens=12*field["clip_from_view"][5]
        # Native volume integration starts at the optical camera, not its
        # opaque near plane. Cycles offsets ray origins by clip_start.
        camera.data.clip_start=1e-6;camera.data.clip_end=1e6;scene.camera=camera
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
                or not 0 < actual_camera["near_m"] <= 1.1e-6
                or len(mesh.loop_triangles)!=12 or obj.matrix_world!=Matrix.Identity(4)):
            raise ValueError("actual Cycles camera or unscaled carrier differs")
        metrics=compare_hdr(original/"linear.rgba16f",rgba)
        metrics["interpretation"]="Native raster/midpoint/half-float versus Cycles stochastic volume transport and pixel reconstruction; diagnostic only"
        center_index=(64*129+64)*4
        reports[case]={"metrics":metrics,"center_linear_rgba":list(rgba[center_index:center_index+4]),
                       "actual_camera":actual_camera,"carrier_triangles":len(mesh.loop_triangles),
                       "axial_representation":"constant coefficients" if field["id"] == ID else "48 frozen f32 samples in two packed linear Non-Color float textures; uniform-coordinate spacing differs from rounded native z by at most 2e-7 of length",
                       "material_node":"ShaderNodeVolumeCoefficients","scatter_coefficients":list(volume.inputs["Scatter Coefficients"].default_value),
                       "artifacts_sha256":{p.name:sha256(p) for p in destination.iterdir() if p.is_file()}}
        if field["id"] in (ENGINE_ID, MOTION_ID):
            reports[case]["steps16_metrics"] = compare_hdr(original/"steps16.rgba16f",rgba)
    if source_state(ROOT)!=source or sha256(directory/"capture.json")!=identity or load_inputs(directory)!=record:
        raise ValueError("adapter source or native inputs changed during comparison")
    report={"id":record["id"],"engine":{"name":"Blender/Cycles","version":bpy.app.version_string,"build_hash":bpy.app.build_hash.decode()},
            "source":source,"tool_sha256":sha256(Path(__file__)),"native_capture_sha256":identity,"native_source":record["source"],
            "settings":{"device":"CPU","samples":samples,"seed":0,"volume_step_rate":0.1,"volume_max_steps":4096,"volume_biased":False,"scattering":0,
                        "volume_ray_start":"native optical camera (t0=0); Cycles minimum clip_start=1e-6 m, not opaque projection near=0.1 m",
                        "alpha_comparison":"RGB only; Cycles opaque film alpha is not transmittance","display":"shared Reinhard/sRGB, gain=1; native raster vs Cycles pixel reconstruction"},
            "cases":reports,"acceptance":{"physical":"frozen field transport only; model-generated engine cases are not optical calibration","visual":"not evaluated","performance":"not comparable","temporal":"not evaluated"}}
    (output/"comparison.json").write_text(json.dumps(report,indent=2)+"\n")
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__); commands=parser.add_subparsers(dest="command",required=True)
    native=commands.add_parser("capture");native.add_argument("--output",type=Path,required=True)
    native.add_argument("--profile",choices=("dev","release"),default="dev");native.add_argument("--allow-dirty",action="store_true");native.add_argument("--suite",choices=("analytic","compiled","motion"),default="analytic")
    external=commands.add_parser("cycles");external.add_argument("capture",type=Path);external.add_argument("--output",type=Path,required=True)
    external.add_argument("--samples",type=int,default=256)
    args=parser.parse_args(sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else None)
    if args.command=="capture":capture(args.output,args.profile,args.allow_dirty,args.suite)
    else:cycles(args.capture,args.output,args.samples)
