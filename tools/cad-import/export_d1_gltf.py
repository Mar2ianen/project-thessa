"""Convert the FreeCAD-derived D1 parts into an animated glTF preview."""

from pathlib import Path
import json
import math

import bpy
from mathutils import Matrix, Vector


ROOT = Path('/home/chechulin/Projects/project-thessa')
ASSET = ROOT / 'assets/models/thessa-d1-docking-port'
OUT = ASSET / 'thessa_d1_docking_port.glb'
PREVIEW = ASSET / 'thessa_d1_docking_port_preview.png'

PETAL_ANGLES_DEG = (24.0, 36.0, 144.0, 156.0, -96.0, -84.0)
PETAL_PIVOTS = [
    (550.0 * math.cos(math.radians(angle)), 550.0 * math.sin(math.radians(angle)), -68.0)
    for angle in PETAL_ANGLES_DEG
]


def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for datablocks in (bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.cameras, bpy.data.lights):
        for block in list(datablocks):
            if block.users == 0:
                datablocks.remove(block)


def material(name, color, metallic=0.0, roughness=0.5):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1.0)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*color, 1.0)
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = roughness
    return mat


def import_mesh(path, name, mat):
    before = set(bpy.context.scene.objects)
    bpy.ops.wm.obj_import(filepath=str(path), forward_axis='X', up_axis='Z')
    created = [obj for obj in bpy.context.scene.objects if obj not in before and obj.type == 'MESH']
    if not created:
        raise RuntimeError(f'OBJ import produced no mesh: {path}')
    bpy.ops.object.select_all(action='DESELECT')
    for obj in created:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = created[0]
    if len(created) > 1:
        bpy.ops.object.join()
    obj = bpy.context.view_layer.objects.active
    obj.name = name
    obj.data.name = f'{name}_mesh'
    obj.data.materials.append(mat)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    # Blender's OBJ importer keeps source vertices in millimetres but adds a
    # display-axis rotation.  The source STEP frame is already +Z-up/+X-forward;
    # clear only that importer transform so source coordinates remain unchanged.
    obj.matrix_basis = Matrix.Identity(4)
    return obj


def set_origin_parent(obj, parent, pivot_mm):
    pivot = Vector(pivot_mm)
    obj.parent = parent
    obj.matrix_parent_inverse = Matrix.Identity(4)
    obj.location = -pivot


def animate_petal(pivot, theta):
    tangent = Vector((-math.sin(theta), math.cos(theta), 0.0)).normalized()
    pivot.rotation_mode = 'AXIS_ANGLE'
    for frame, angle in ((1, 0.0), (30, 0.4363323129985824), (60, 0.0)):
        pivot.rotation_axis_angle = (angle, tangent.x, tangent.y, tangent.z)
        pivot.keyframe_insert('rotation_axis_angle', frame=frame)


def look_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat('-Z', 'Y').to_euler()


def main():
    clear_scene()
    material_contract = json.loads((ASSET / 'materials.json').read_text(encoding='utf-8'))['materials']
    structural = material_contract['structural_metal']
    capture = material_contract['capture_hardware']
    fixed_mat = material(
        'd1.structural_metal',
        tuple(structural['base_color_linear']),
        metallic=structural['metallic'],
        roughness=structural['roughness'],
    )
    petal_mat = material(
        'd1.capture_hardware',
        tuple(capture['base_color_linear']),
        metallic=capture['metallic'],
        roughness=capture['roughness'],
    )

    root = bpy.data.objects.new('Thessa_D1_Docking_Port', None)
    bpy.context.collection.objects.link(root)
    root.scale = (0.001, 0.001, 0.001)

    fixed = import_mesh(ASSET / 'fixed_structure.obj', 'FixedStructure', fixed_mat)
    fixed.parent = root
    fixed.matrix_parent_inverse = Matrix.Identity(4)

    for number, pivot_mm in enumerate(PETAL_PIVOTS, 1):
        petal = import_mesh(ASSET / f'petal_{number:02d}.obj', f'SoftCapturePetal_{number:02d}', petal_mat)
        pivot = bpy.data.objects.new(f'SoftCapturePetal_{number:02d}_Hinge', None)
        bpy.context.collection.objects.link(pivot)
        pivot.location = Vector(pivot_mm)
        pivot.parent = root
        pivot.matrix_parent_inverse = Matrix.Identity(4)
        set_origin_parent(petal, pivot, pivot_mm)
        theta = math.radians(PETAL_ANGLES_DEG[number - 1])
        animate_petal(pivot, theta)

    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = 60
    scene.render.fps = 30
    scene.render.engine = 'BLENDER_EEVEE'
    scene.render.resolution_x = 768
    scene.render.resolution_y = 768
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.filepath = str(PREVIEW)
    scene.world.color = (0.015, 0.02, 0.03)

    camera_data = bpy.data.cameras.new('PreviewCamera')
    camera = bpy.data.objects.new('PreviewCamera', camera_data)
    bpy.context.collection.objects.link(camera)
    camera.location = (2.3, -2.4, 1.65)
    camera.data.lens = 52
    look_at(camera, (0.0, 0.0, -0.02))
    scene.camera = camera

    light_data = bpy.data.lights.new('Key', 'AREA')
    light_data.energy = 1100
    light_data.shape = 'DISK'
    light_data.size = 3.0
    key = bpy.data.objects.new('Key', light_data)
    bpy.context.collection.objects.link(key)
    key.location = (1.6, -1.4, 2.6)
    look_at(key, (0.0, 0.0, 0.0))

    fill_data = bpy.data.lights.new('Fill', 'AREA')
    fill_data.energy = 500
    fill_data.size = 2.0
    fill = bpy.data.objects.new('Fill', fill_data)
    bpy.context.collection.objects.link(fill)
    fill.location = (-1.8, 1.1, 1.2)
    look_at(fill, (0.0, 0.0, 0.0))

    scene.frame_set(30)
    bpy.ops.render.render(write_still=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(ASSET / 'thessa_d1_docking_port_preview.blend'))
    bpy.ops.export_scene.gltf(
        filepath=str(OUT),
        export_format='GLB',
        export_animations=True,
        export_skins=False,
        use_selection=False,
    )
    print(f'generated {OUT}')


if __name__ == '__main__':
    main()
