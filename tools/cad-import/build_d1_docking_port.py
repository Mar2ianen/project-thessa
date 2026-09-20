"""Build the first D1 docking-port asset bundle from the supplied STEP fixture.

This is an offline FreeCAD/OpenCascade tool.  It deliberately keeps STEP and
FreeCAD at the tooling boundary; the game-facing files are derived products.
The six visible, radially repeated guide/petal source solids at STEP indices
17, 20, 25, 28, 33, and 36 are used as the provisional soft-capture petals
until semantic feature recognition is available.
"""

import json
import math
import os
from pathlib import Path

import FreeCAD as App
import MeshPart
import Part


REPO = Path('/home/chechulin/Projects/project-thessa')
SOURCE = Path('/home/chechulin/Downloads/thessa_d1_v2_2_free.step')
OUT = REPO / 'assets/models/thessa-d1-docking-port'
PETAL_INDICES = [17, 20, 25, 28, 33, 36]
PETAL_ANGLES_DEG = [24.0, 36.0, 144.0, 156.0, -96.0, -84.0]
OPEN_ANGLE_DEG = 25.0
PETAL_HINGE_RADIUS_MM = 550.0
PETAL_HINGE_Z_MM = -68.0


def mesh_shape(shape, path):
    mesh = MeshPart.meshFromShape(
        Shape=shape,
        LinearDeflection=0.5,
        AngularDeflection=0.35,
        Relative=False,
    )
    mesh.write(str(path))


def compound(shapes):
    return Part.makeCompound([shape for shape in shapes if shape and not shape.isNull()])


def placement_for_petal(index, angle_deg):
    theta = math.radians(PETAL_ANGLES_DEG[PETAL_INDICES.index(index)])
    pivot = App.Vector(
        PETAL_HINGE_RADIUS_MM * math.cos(theta),
        PETAL_HINGE_RADIUS_MM * math.sin(theta),
        PETAL_HINGE_Z_MM,
    )
    tangent = App.Vector(-math.sin(theta), math.cos(theta), 0.0)
    rotation = App.Rotation(tangent, angle_deg)
    return App.Placement(pivot, rotation).multiply(
        App.Placement(App.Vector(-pivot.x, -pivot.y, -pivot.z), App.Rotation())
    )


def set_petal_pose(objects, angle_deg):
    for index, obj in zip(PETAL_INDICES, objects):
        obj.Placement = placement_for_petal(index, angle_deg)


def add_string(obj, name, group, value):
    obj.addProperty('App::PropertyString', name, group)
    setattr(obj, name, value)


def make_document():
    doc = App.newDocument('thessa_d1_docking_port')
    Part.insert(str(SOURCE), doc.Name)
    doc.recompute()
    imported = [obj for obj in doc.Objects if hasattr(obj, 'Shape') and not obj.Shape.isNull()]
    if len(imported) != 139:
        raise RuntimeError(f'expected 139 imported solids, got {len(imported)}')

    root = doc.addObject('App::Part', 'ThessaD1DockingPort')
    root.Label = 'Thessa D1 docking port (STEP-derived assembly)'
    add_string(root, 'SemanticRole', 'Asset', 'docking_port')
    add_string(root, 'PortClass', 'Asset', 'D1')
    add_string(root, 'SourceFormat', 'Asset', 'STEP / OpenCascade BRep')
    add_string(root, 'SourceFixture', 'Asset', SOURCE.name)
    add_string(root, 'CoordinateSystem', 'Asset', 'source STEP frame; millimetres; Z up')
    add_string(root, 'SemanticStatus', 'Asset', 'provisional mechanical mapping')

    fixed_group = doc.addObject('App::Part', 'FixedStructure')
    fixed_group.Label = 'Fixed structure and non-semantic source solids'
    root.addObject(fixed_group)
    moving_group = doc.addObject('App::Part', 'SoftCaptureMechanism')
    moving_group.Label = 'Soft-capture petals (provisional source mapping)'
    root.addObject(moving_group)

    fixed = []
    petals = []
    for index, obj in enumerate(imported):
        obj.Label = f'STEP solid {index:03d}'
        if index in PETAL_INDICES:
            obj.Label = f'Soft-capture petal {index - PETAL_INDICES[0] + 1:02d}'
            moving_group.addObject(obj)
            obj.addProperty('App::PropertyInteger', 'SourceSolidIndex', 'CAD')
            obj.SourceSolidIndex = index
            obj.addProperty('App::PropertyAngle', 'OpenAngle', 'Motion')
            obj.OpenAngle = OPEN_ANGLE_DEG
            petals.append(obj)
        else:
            fixed_group.addObject(obj)
            fixed.append(obj)

    controller = doc.addObject('App::FeaturePython', 'SoftCaptureController')
    controller.Label = 'Soft-capture pose controller'
    root.addObject(controller)
    controller.addProperty('App::PropertyAngle', 'PoseAngle', 'Motion')
    controller.PoseAngle = 0.0
    controller.addProperty('App::PropertyAngle', 'OpenPose', 'Motion')
    controller.OpenPose = OPEN_ANGLE_DEG
    controller.addProperty('App::PropertyString', 'Mechanism', 'Motion')
    controller.Mechanism = 'six radial petals, hinge axis tangent to capture ring'
    controller.addProperty('App::PropertyString', 'Authority', 'Motion')
    controller.Authority = 'runtime joint/actuator state; render mesh is derived'
    controller.addProperty('App::PropertyString', 'Status', 'Motion')
    controller.Status = 'MVP animation fixture; exact actuator limits TBD'

    doc.recompute()
    return doc, imported, fixed, petals, root, controller


def write_manifest(bb, source_sha256):
    manifest = {
        'schema': 'thessa.cad_asset.v0',
        'asset_id': 'thessa.docking_port.d1.v2_2',
        'semantic_role': 'docking_port',
        'port_class': 'D1',
        'source': {
            'file': SOURCE.name,
            'sha256': source_sha256,
            'provenance': 'user-supplied local STEP fixture; license not recorded yet',
            'canonical_layer': 'STEP reference / normalized BRep importer input',
        },
        'units': 'millimetres in source products; metres in glTF',
        'coordinate_system': {
            'source': 'STEP source frame',
            'up': '+Z',
            'forward': '+X (provisional until vehicle integration)',
        },
        'audit': {
            'solids': 139,
            'bbox_mm': [round(bb.XLength, 3), round(bb.YLength, 3), round(bb.ZLength, 3)],
            'bbox_min_mm': [round(bb.XMin, 3), round(bb.YMin, 3), round(bb.ZMin, 3)],
            'bbox_max_mm': [round(bb.XMax, 3), round(bb.YMax, 3), round(bb.ZMax, 3)],
        },
        'derived_products': {
            'freecad_closed': 'thessa_d1_docking_port_closed.FCStd',
            'freecad_open': 'thessa_d1_docking_port_open.FCStd',
            'visual_gltf': 'thessa_d1_docking_port.glb',
            'fixed_obj': 'fixed_structure.obj',
            'petal_objs': [f'petal_{i:02d}.obj' for i in range(1, 7)],
        },
        'mechanics': {
            'state_machine': ['free', 'soft_capture', 'aligned', 'hard_dock', 'outer_structure_engaged', 'pressure_equalized'],
            'provisional_moving_parts': [{
                'id': 'soft_capture_petals',
                'count': 6,
                'source_solid_indices': PETAL_INDICES,
                'hinge_angles_deg': PETAL_ANGLES_DEG,
                'hinge_radius_mm': PETAL_HINGE_RADIUS_MM,
                'hinge_z_mm': PETAL_HINGE_Z_MM,
                'open_angle_deg': OPEN_ANGLE_DEG,
                'mapping_status': 'provisional; validate against mechanical feature recognition',
            }],
            'tbd': [
                'clear passage and seal diameter',
                'capture velocity limits',
                'stiffness, damping, and structural load limits',
                'service/power/data/fluid pinout',
            ],
        },
        'runtime_contract': {
            'render': 'derived adaptive CAD/RCBT surface cache; current MVP ships glTF preview',
            'collision': 'separate product; do not derive authoritative behavior from view tessellation',
            'actuation': 'explicit joint/actuator graph; controller metadata is not physics authority',
        },
        'materials': {
            'structural_metal': {
                'semantic_id': 'd1.structural_metal',
                'base_color_linear': [0.22, 0.28, 0.34],
                'metallic': 0.80,
                'roughness': 0.32,
                'contact_friction': 0.65,
                'contact_restitution': 0.02,
            },
            'capture_hardware': {
                'semantic_id': 'd1.capture_hardware',
                'base_color_linear': [0.82, 0.34, 0.08],
                'metallic': 0.55,
                'roughness': 0.38,
                'contact_friction': 0.75,
                'contact_restitution': 0.01,
            },
            'seal_elastomer': {
                'semantic_id': 'd1.seal_elastomer',
                'base_color_linear': [0.04, 0.045, 0.05],
                'metallic': 0.0,
                'roughness': 0.72,
                'contact_friction': 0.90,
                'contact_restitution': 0.0,
            },
        },
    }
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')


def write_physics_contract():
    parts = []
    ring_radius = 0.56
    for i in range(8):
        theta = 2.0 * math.pi * i / 8.0
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        parts.append(
            f'''[[collision_parts]]
id = "capture_ring_proxy_{i + 1:02d}"
shape = "cuboid"
position_body_m = [{ring_radius * math.cos(theta):.6f}, {ring_radius * math.sin(theta):.6f}, -0.100000]
orientation_body_xyzw = [0.0, 0.0, {qz:.9f}, {qw:.9f}]
half_extents_m = [0.080000, 0.260000, 0.080000]
material = "structural_metal"
'''
        )
    physics_text = '''# Provisional Thessa D1 docking-port physics contract.
# Units are SI metres / seconds / kilograms.
# The values marked provisional are integration fixtures, not final load ratings.
schema = "thessa.docking_port.physics.v0"
asset_id = "thessa.docking_port.d1.v2_2"
solver = "rapier3d-f64"
status = "MVP contact and mechanism contract; calibrate against structural model"

[rapier]
gravity_scale = 0.0
ccd_enabled = true
can_sleep = true
density_source = "Thessa RigidBodyProperties, not Rapier collider density"

[docking_interface]
id = "d1_pressurized_interface"
family = "standard_pressurized"
port_class = "D1"
common_core = true
local_position_m = [0.0, 0.0, 0.0]
local_orientation_xyzw = [0.0, 0.0, 0.0, 1.0]
capture_velocity_limit_mps = 0.05 # provisional
alignment_tolerance_rad = 0.035 # provisional

[collision_materials.structural_metal]
friction = 0.65
restitution = 0.02

[collision_materials.capture_hardware]
friction = 0.75
restitution = 0.01

[collision_materials.seal_elastomer]
friction = 0.90
restitution = 0.0

# The eight convex blocks leave the common passage open. They are a
# conservative contact proxy and must not define the final seal geometry.
''' + ''.join(parts) + '''
[moving_mechanism]
id = "soft_capture_petals"
joint_type = "revolute"
count = 6
source_solid_indices = [17, 20, 25, 28, 33, 36]
hinge_angles_deg = [24.0, 36.0, 144.0, 156.0, -96.0, -84.0]
hinge_radius_m = 0.536
hinge_z_m = -0.078
closed_angle_rad = 0.0
open_angle_rad = 0.436332313
actuator_authority = "Thessa mechanism/actuator graph"
rapier_joint_authority = "future revolute-joint bridge; current public seam exposes fixed docking joints"

[mechanical_limits]
axial_force_limit_n = 0.0 # TBD
shear_force_limit_n = 0.0 # TBD
bending_moment_limit_nm = 0.0 # TBD
torsional_moment_limit_nm = 0.0 # TBD
'''
    physics_text = physics_text.replace(
        'hinge_radius_m = 0.536',
        f'hinge_radius_m = {PETAL_HINGE_RADIUS_MM / 1000.0:.3f}',
    ).replace(
        'hinge_z_m = -0.078',
        f'hinge_z_m = {PETAL_HINGE_Z_MM / 1000.0:.3f}',
    )
    (OUT / 'physics.toml').write_text(physics_text, encoding='utf-8')

    materials = {
        'schema': 'thessa.material_contract.v0',
        'asset_id': 'thessa.docking_port.d1.v2_2',
        'coordinate_independent': True,
        'materials': {
            'structural_metal': {
                'semantic_id': 'd1.structural_metal',
                'base_color_linear': [0.22, 0.28, 0.34],
                'metallic': 0.80,
                'roughness': 0.32,
                'contact_friction': 0.65,
                'contact_restitution': 0.02,
            },
            'capture_hardware': {
                'semantic_id': 'd1.capture_hardware',
                'base_color_linear': [0.82, 0.34, 0.08],
                'metallic': 0.55,
                'roughness': 0.38,
                'contact_friction': 0.75,
                'contact_restitution': 0.01,
            },
            'seal_elastomer': {
                'semantic_id': 'd1.seal_elastomer',
                'base_color_linear': [0.04, 0.045, 0.05],
                'metallic': 0.0,
                'roughness': 0.72,
                'contact_friction': 0.90,
                'contact_restitution': 0.0,
            },
        },
    }
    (OUT / 'materials.json').write_text(json.dumps(materials, indent=2) + '\n', encoding='utf-8')


def main():
    if not SOURCE.exists():
        raise FileNotFoundError(SOURCE)
    OUT.mkdir(parents=True, exist_ok=True)
    doc, imported, fixed, petals, root, controller = make_document()
    all_shape = compound([obj.Shape for obj in imported])
    fixed_shape = compound([obj.Shape for obj in fixed])
    bb = all_shape.BoundBox

    controller.PoseAngle = 0.0
    set_petal_pose(petals, 0.0)
    doc.recompute()
    doc.saveAs(str(OUT / 'thessa_d1_docking_port_closed.FCStd'))
    mesh_shape(fixed_shape, OUT / 'fixed_structure.obj')
    for number, petal in enumerate(petals, 1):
        mesh_shape(petal.Shape, OUT / f'petal_{number:02d}.obj')

    controller.PoseAngle = OPEN_ANGLE_DEG
    set_petal_pose(petals, OPEN_ANGLE_DEG)
    doc.recompute()
    doc.saveAs(str(OUT / 'thessa_d1_docking_port_open.FCStd'))

    (OUT / 'geometry_report.txt').write_text(
        f'objects={len(imported)}\n'
        f'solids={len(all_shape.Solids)}\n'
        f'bbox_mm={bb.XLength:.3f},{bb.YLength:.3f},{bb.ZLength:.3f}\n'
        f'bbox_min_mm={bb.XMin:.3f},{bb.YMin:.3f},{bb.ZMin:.3f}\n'
        f'bbox_max_mm={bb.XMax:.3f},{bb.YMax:.3f},{bb.ZMax:.3f}\n',
        encoding='utf-8',
    )
    import hashlib
    source_sha256 = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    write_manifest(bb, source_sha256)
    write_physics_contract()
    print(f'generated {OUT}')


if __name__ == '__main__':
    main()
