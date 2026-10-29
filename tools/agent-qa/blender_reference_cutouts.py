"""Trusted Blender-side producer for the explicitly derived B02 core-cutout-v1 dataset.
Original Blender shader graphs are not claimed equivalent to this material variant.
"""
import hashlib
import json
import os
import struct
from pathlib import Path
from array import array

import bpy
source = Path(bpy.data.filepath)
asset = source.name.removesuffix('_1k.blend')
assert asset in {'fern_02', 'moss_01'}
output = Path(os.environ['THESSA_ASSET_EXPORT'])
output.mkdir(parents=True, exist_ok=True)
expected = 6232 if asset == 'fern_02' else 245438
objects = [o for o in bpy.context.scene.objects if o.type == 'MESH' and (asset == 'fern_02' or o.name == 'moss_01_geometry_nodes')]
assert len(objects) == (4 if asset == 'fern_02' else 1)
report = {'variant': 'core-cutout-v1', 'asset': asset, 'source_blend_sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'blender_version': bpy.app.version_string, 'blender_build_hash': bpy.app.build_hash.decode(), 'frame': bpy.context.scene.frame_current, 'source_unit_scale': bpy.context.scene.unit_settings.scale_length, 'coordinate_conversion': 'Blender Z-up to glTF right-handed Y-up; metres; existing object world transforms retained', 'material_policy': 'Explicit derived core metallic-roughness/cutout variant, not the original Blender shader graph. Raw scanned base-color/roughness/normal/alpha maps; default glTF dielectric IOR 1.5 and specular response; source translucency/mix/add/hue graph omitted and declared unavailable. Dithered coverage replaced by MASK using the stored 0.5 alpha threshold, not original dithered-render equivalence.', 'tangent_policy': 'No authored TANGENT accessor exported; UVs and vertex normals retained. Native per-fragment UV-derivative tangent basis; Blender/MikkTSpace equivalence not claimed.', 'objects': [], 'materials': [], 'source_images': []}
assert bpy.context.scene.unit_settings.scale_length == 1.0

def nodes_deep(tree, seen=None):
    seen = set() if seen is None else seen
    if tree is None or tree.name in seen:
        return []
    seen.add(tree.name)
    result = list(tree.nodes)
    for n in tree.nodes:
        if n.type == 'GROUP':
            result.extend(nodes_deep(n.node_tree, seen))
    return result

def evaluated_materials(obj):
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = evaluated.to_mesh()
    result = list(mesh.materials)
    evaluated.to_mesh_clear()
    return result
material_map = {}
expected_modes = {}
for obj in objects:
    for original in evaluated_materials(obj):
        if original.name in material_map:
            continue
        nodes = nodes_deep(original.node_tree)
        for n in nodes:
            if n.type == 'MAPPING':
                for key, identity in [('Location', [0, 0, 0]), ('Rotation', [0, 0, 0]), ('Scale', [1, 1, 1])]:
                    assert all((abs(a - b) < 1e-08 for a, b in zip(n.inputs[key].default_value, identity))), 'nonidentity source UV mapping requires an explicit bake'
        simple = original.name == 'moss_01_sphere'
        pnodes = [n for n in nodes if n.type == 'BSDF_PRINCIPLED']
        strengths = {float(n.inputs['Strength'].default_value) for n in nodes if n.type == 'NORMAL_MAP'}
        assert len(strengths) <= 1
        strength = next(iter(strengths), 1.0)
        material = bpy.data.materials.new(original.name + '__B02_CoreCutoutV1')
        material.use_nodes = True
        material.use_backface_culling = original.use_backface_culling
        tree = material.node_tree
        bsdf = tree.nodes.get('Principled BSDF')
        bsdf.inputs['IOR'].default_value = 1.5
        bsdf.inputs['Specular IOR Level'].default_value = 0.5
        bsdf.inputs['Metallic'].default_value = float(pnodes[0].inputs['Metallic'].default_value) if simple else 0.0
        bsdf.inputs['Roughness'].default_value = float(pnodes[0].inputs['Roughness'].default_value) if simple else 1.0
        source_values = [{key: list(node.inputs[key].default_value) if hasattr(node.inputs[key].default_value, '__len__') else float(node.inputs[key].default_value) for key in ['Base Color', 'Metallic', 'Roughness', 'IOR', 'Alpha', 'Specular IOR Level', 'Transmission Weight', 'Subsurface Weight']} for node in pnodes]
        entry = {'source_principled_parameters': source_values, 'source': original.name, 'derived': material.name, 'source_render_method': getattr(original, 'surface_render_method', None), 'source_alpha_threshold': float(original.alpha_threshold), 'normal_scale': strength, 'dielectric_ior': 1.5, 'specular_ior_level': 0.5, 'metallic': float(bsdf.inputs['Metallic'].default_value), 'roughness_factor': float(bsdf.inputs['Roughness'].default_value), 'double_sided': not material.use_backface_culling}
        if simple:
            bsdf.inputs['Base Color'].default_value = pnodes[0].inputs['Base Color'].default_value
            expected_modes[material.name] = ('OPAQUE', None)
        else:

            def texture(suffix, color_space):
                path = source.parent / 'textures' / f'{asset}_{suffix}_1k'
                matches = list(path.parent.glob(path.name + '.*'))
                assert len(matches) == 1
                image = bpy.data.images.load(str(matches[0]), check_existing=True)
                assert image.colorspace_settings.name == color_space
                node = tree.nodes.new('ShaderNodeTexImage')
                node.image = image
                if not any((e['path'] == str(matches[0].relative_to(source.parent)) for e in report['source_images'])):
                    pixels = array('f', [0]) * len(image.pixels)
                    image.pixels.foreach_get(pixels)
                    report['source_images'].append({'path': str(matches[0].relative_to(source.parent)), 'sha256': hashlib.sha256(matches[0].read_bytes()).hexdigest(), 'color_space': color_space, 'dimensions': list(image.size), 'decoded_channel_min': min(pixels), 'decoded_channel_max': max(pixels)})
                return node
            color = texture('diff', 'sRGB')
            rough = texture('rough', 'Non-Color')
            normal = texture('nor_gl', 'Non-Color')
            alpha = texture('alpha', 'Non-Color')
            tree.links.new(color.outputs['Color'], bsdf.inputs['Base Color'])
            tree.links.new(rough.outputs['Color'], bsdf.inputs['Roughness'])
            tree.links.new(alpha.outputs['Color'], bsdf.inputs['Alpha'])
            nmap = tree.nodes.new('ShaderNodeNormalMap')
            nmap.inputs['Strength'].default_value = strength
            tree.links.new(normal.outputs['Color'], nmap.inputs['Color'])
            tree.links.new(nmap.outputs['Normal'], bsdf.inputs['Normal'])
            material.surface_render_method = 'DITHERED'
            expected_modes[material.name] = ('MASK', float(original.alpha_threshold))
        material_map[original.name] = material
        report['materials'].append(entry)
# Freeze the evaluated mesh; source files and original shader graphs stay intact.
deps = bpy.context.evaluated_depsgraph_get()
frozen = []
triangles = 0
bpy.ops.object.select_all(action='DESELECT')
for obj in objects:
    evaluated = obj.evaluated_get(deps)
    mesh = bpy.data.meshes.new_from_object(evaluated, preserve_all_data_layers=True, depsgraph=deps)
    mesh.calc_loop_triangles()
    triangles += len(mesh.loop_triangles)
    item = {'source_object': obj.name, 'triangles': len(mesh.loop_triangles), 'vertices': len(mesh.vertices), 'uv_layers': [uv.name for uv in mesh.uv_layers], 'world_matrix': [list(row) for row in obj.matrix_world]}
    report['objects'].append(item)
    for i, material in enumerate(list(mesh.materials)):
        mesh.materials[i] = material_map[material.name]
    target = bpy.data.objects.new(obj.name + '__reference_frozen', mesh)
    target.matrix_world = obj.matrix_world.copy()
    bpy.context.scene.collection.objects.link(target)
    target.select_set(True)
    frozen.append(target)
assert triangles == expected, (triangles, expected)
bpy.context.view_layer.objects.active = frozen[0]
path = output / f'{asset}.glb'
kwargs = {'filepath': str(path), 'export_format': 'GLB', 'use_selection': True, 'export_apply': False, 'export_yup': True, 'export_normals': True, 'export_texcoords': True, 'export_tangents': False, 'export_materials': 'EXPORT', 'export_image_format': 'AUTO', 'export_extras': False}
report['export_options'] = kwargs.copy()
report['export_options']['filepath'] = path.name
bpy.ops.export_scene.gltf(**kwargs)
raw = path.read_bytes()
assert raw[:4] == b'glTF'
n, kind = struct.unpack_from('<II', raw, 12)
assert kind == 0x4E4F534A
doc = json.loads(raw[20:20 + n])
offset = 20 + n
length, kind = struct.unpack_from('<II', raw, offset)
assert kind == 0x004E4942
binary = raw[offset + 8:offset + 8 + length]
report['exporter_material_delta'] = []
for material in doc['materials']:
    mode, cutoff = expected_modes[material['name']]
    before = material.get('alphaMode', 'OPAQUE')
    if mode == 'MASK':
        assert before in ['BLEND', 'MASK'], 'exporter lost source alpha'
    material['alphaMode'] = mode
    if cutoff is not None:
        material['alphaCutoff'] = cutoff
    report['exporter_material_delta'].append({'material': material['name'], 'exported_alpha_mode': before, 'benchmark_alpha_mode': mode, 'benchmark_cutoff': cutoff})
    for name, extension in list(material.get('extensions', {}).items()):
        assert name == 'KHR_materials_ior' and extension == {'ior': 1.5} or (name == 'KHR_materials_specular' and all((key == 'specularFactor' and value == 1 or (key == 'specularColorFactor' and value == [1, 1, 1]) for key, value in extension.items()))), ('unexpected material extension', name, extension)
        del material['extensions'][name]
    if material.get('extensions') == {}:
        del material['extensions']
assert set(doc.get('extensionsUsed', [])) <= {'KHR_materials_ior', 'KHR_materials_specular'}
assert set(doc.get('extensionsRequired', [])) <= {'KHR_materials_ior', 'KHR_materials_specular'}
doc.pop('extensionsUsed', None)
doc.pop('extensionsRequired', None)
encoded = json.dumps(doc, sort_keys=True, separators=(',', ':')).encode()
encoded += b' ' * (-len(encoded) % 4)
result = struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(encoded) + 8 + len(binary)) + struct.pack('<II', len(encoded), 0x4E4F534A) + encoded + struct.pack('<II', len(binary), 0x004E4942) + binary
path.write_bytes(result)
report['triangles'] = sum((doc['accessors'][primitive['indices']]['count'] // 3 for mesh in doc['meshes'] for primitive in mesh['primitives']))
assert report['triangles'] == expected
report['glb_bytes'] = len(result)
report['glb_sha256'] = hashlib.sha256(result).hexdigest()
report['source_geometry_expected_triangles'] = expected
report['converter_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
report['images'] = [{'mime_type': image.get('mimeType'), 'bytes': doc['bufferViews'][image['bufferView']]['byteLength']} for image in doc.get('images', [])]
(output / 'conversion.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({'asset': asset, 'variant': 'core-cutout-v1', 'triangles': report['triangles'], 'glb_bytes': len(result), 'sha256': report['glb_sha256']}))
