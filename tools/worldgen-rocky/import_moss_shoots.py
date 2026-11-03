#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Pack three pinned CC0 Moss 01 shoot forms; offline Pillow, no runtime download."""
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
from PIL import Image

source, destination = map(Path, sys.argv[1:3])
expected = {
    'moss_01_1k.gltf': '1baba498fd3936673609dda5ecfe960a',
    'moss_01.bin': 'cdb092878a81b3208a162d79546892fe',
    'moss_01_diff_1k.jpg': '5768f973a7d6d2846a2e76a6b0669a83',
    'moss_01_alpha_1k.png': 'e55f1cca390d93f23980a5fdbe1dc705',
    'moss_01_nor_gl_1k.jpg': 'faf161f708abe274db1b8087800fa08b',
    'moss_01_arm_1k.jpg': 'e3b6adf32db7076885afe0e179cc1850',
}
base_url = 'https://dl.polyhaven.org/file/ph-assets/Models/'
source_urls = {
    'moss_01_1k.gltf': base_url+'gltf/1k/moss_01/moss_01_1k.gltf',
    'moss_01.bin': base_url+'gltf/4k/moss_01/moss_01.bin',
    'moss_01_diff_1k.jpg': base_url+'jpg/1k/moss_01/moss_01_diff_1k.jpg',
    'moss_01_alpha_1k.png': base_url+'png/1k/moss_01/moss_01_alpha_1k.png',
    'moss_01_nor_gl_1k.jpg': base_url+'jpg/1k/moss_01/moss_01_nor_gl_1k.jpg',
    'moss_01_arm_1k.jpg': base_url+'jpg/1k/moss_01/moss_01_arm_1k.jpg',
}
raw = {name: (source / name).read_bytes() for name in expected}
for name, data in raw.items():
    assert hashlib.md5(data).hexdigest() == expected[name], name
model = json.loads(raw['moss_01_1k.gltf'])
# The glTF is the source-organ gallery, not the dense Geometry Nodes preview.
forms = [7, 8, 9]
model['meshes'] = [model['meshes'][i] for i in forms]
model['nodes'] = [dict(mesh=i, name=f'moss-shoot-{i}') for i in range(3)]
model['scenes'] = [dict(nodes=[0, 1, 2])]
model['scene'] = 0
model.pop('extensionsUsed', None)
model.pop('extensionsRequired', None)
for material in model['materials']:
    # Retain the supported standard PBR maps; use the native dielectric prior.
    material.pop('extensions', None)
    material['alphaMode'] = 'MASK'
    material['alphaCutoff'] = 0.5
rgb = Image.open(io.BytesIO(raw['moss_01_diff_1k.jpg'])).convert('RGBA')
alpha = Image.open(io.BytesIO(raw['moss_01_alpha_1k.png'])).convert('L')
assert rgb.size == alpha.size == (1024, 1024)
rgb.putalpha(alpha)
png = io.BytesIO()
rgb.save(png, format='PNG', compress_level=9)
binary = bytearray(raw['moss_01.bin'])
images = [(raw['moss_01_nor_gl_1k.jpg'], 'image/jpeg'),
          (png.getvalue(), 'image/png'), (raw['moss_01_arm_1k.jpg'], 'image/jpeg')]
for image, (payload, mime) in zip(model['images'], images):
    binary.extend(b'\0' * (-len(binary) % 4))
    view = len(model['bufferViews'])
    model['bufferViews'].append(dict(buffer=0, byteOffset=len(binary), byteLength=len(payload)))
    binary.extend(payload)
    image.pop('uri', None)
    image.update(bufferView=view, mimeType=mime)
model['buffers'] = [dict(byteLength=len(binary))]
text = json.dumps(model, separators=(',', ':')).encode()
text += b' ' * (-len(text) % 4)
binary.extend(b'\0' * (-len(binary) % 4))
result = (struct.pack('<III', 0x46546C67, 2, 28 + len(text) + len(binary))
          + struct.pack('<II', len(text), 0x4E4F534A) + text
          + struct.pack('<II', len(binary), 0x004E4942) + binary)
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_bytes(result)
metadata = dict(source='https://polyhaven.com/a/moss_01', author='Rob Tuytel',
    license='CC0-1.0', source_forms=forms, triangles=[6, 6, 8],
    transforms='gallery node translations removed; source shoot coordinates/UVs retained',
    material_conversion='diffuse JPEG + separate alpha -> RGBA PNG; MASK 0.5; standard normal/ORM retained; unsupported specular/ior extensions removed for native dielectric prior',
    sources={name: dict(url=source_urls[name], md5=expected[name], sha256=hashlib.sha256(data).hexdigest()) for name, data in raw.items()},
    path=str(destination), bytes=len(result), sha256=hashlib.sha256(result).hexdigest())
destination.with_suffix('.provenance.json').write_text(json.dumps(metadata, indent=2)+'\n')
print(json.dumps({k: metadata[k] for k in ['bytes', 'sha256', 'triangles']}))
