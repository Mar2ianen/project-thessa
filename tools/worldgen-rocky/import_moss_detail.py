#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Pack verified CC0 forest-moss source maps. Pillow/NumPy are offline tools."""
import gzip
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image

source = Path(sys.argv[1])
destination = Path(sys.argv[2])
expected = {
    'diffuse': '09903abd88ae5a12932b79f03b0da15f7a952e9b43a4ad041eac31a3b925736d',
    'rough': '3b99db84fcd4d6a2035086de36e06d399a7ad1edcf214335ccd443edd998ad9b',
    'disp': 'c5d626de38deda9fcbdb00f53f1898698bef11d264b8a2bdeb4fdcd42d399e90',
}
url = 'https://dl.polyhaven.org/file/ph-assets/Textures/png/1k/forest_leaves_02/forest_leaves_02_{}_1k.png'
maps = {}
for name, digest in expected.items():
    path = source / (name + '.png')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, path
    maps[name] = np.asarray(Image.open(path))
side = 1024
assert maps['diffuse'].shape == (side, side, 3)
assert maps['rough'].shape == maps['disp'].shape == (side, side)
packed = np.empty((side, side, 6), dtype=np.uint8)
packed[:, :, :3] = maps['diffuse'].astype(np.uint8)
packed[:, :, 3] = maps['rough'].astype(np.uint8)
packed[:, :, 4:] = maps['disp'].astype('<u2').view(np.uint8).reshape(side, side, 2)
raw = b'FG01' + side.to_bytes(4, 'little') + packed.tobytes()
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_bytes(gzip.compress(raw, compresslevel=9, mtime=0))
metadata = dict(path='assets/textures/forest-moss-litter-02.bin.gz', sha256=hashlib.sha256(destination.read_bytes()).hexdigest(), bytes=destination.stat().st_size, decoded_bytes=len(raw), side=side, source_url='https://polyhaven.com/a/forest_leaves_02', license='CC0-1.0', width_m=3.0, height_range_m=0.02, height_is_authored_optical_prior=True, sources={name:dict(url=url.format(name),sha256=digest) for name,digest in expected.items()})
destination.with_name(destination.name.removesuffix('.bin.gz')+'.provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps(metadata,indent=2))
