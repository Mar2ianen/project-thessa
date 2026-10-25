#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Pack pinned CC0 mud/litter maps; image libraries are offline tooling only."""
import gzip
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image

source, destination = map(Path, sys.argv[1:3])
checks = {"diff": "17506c4b1c5000ed5add829f1de825e851fcd8ffc4b8ed16c45b551ade935ea1",
          "rough": "e2c9d83b00cbb851978865b6ad3e3a760d5cd6c522b849a9c6de2cf3cc681628",
          "disp": "9fa035cad924c4429fee2a510aca0e80acfd943b055f86a962a8a0a1f13be9d3"}
maps = {}
for role, expected in checks.items():
    path = source / f"brown_mud_leaves_01_{role}_1k.png"
    assert hashlib.sha256(path.read_bytes()).hexdigest() == expected
    maps[role] = np.asarray(Image.open(path))
assert maps['diff'].dtype == np.uint8 and maps['diff'].shape == (1024, 1024, 3)
assert all(maps[r].dtype == np.uint16 and maps[r].shape == (1024, 1024) for r in ['rough','disp'])
def average(a):
    return (a[0::2,0::2] + a[1::2,0::2] + a[0::2,1::2] + a[1::2,1::2]) * .25
rgb = maps['diff'].astype(float) / 255
linear = average(np.where(rgb <= .04045, rgb / 12.92, ((rgb+.055)/1.055)**2.4))
rgb = np.where(linear <= .0031308, linear*12.92, 1.055*linear**(1/2.4)-.055)
rough = average(maps['rough'].astype(float)) / 65535
height = average(maps['disp'].astype(float))
p = np.empty((512,512,6),np.uint8)
p[:,:,:3] = np.rint(rgb*255).astype(np.uint8)
p[:,:,3] = np.rint(rough*255).astype(np.uint8)
p[:,:,4:] = np.rint(height).astype('<u2').view(np.uint8).reshape(512,512,2)
raw = b'FG01' + (512).to_bytes(4,'little') + p.tobytes()
destination.write_bytes(gzip.compress(raw,compresslevel=9,mtime=0))
# Report the exact quantized payload mean, matching runtime RGB decoding.
v = p[:,:,:3].astype(float)/255
v = np.where(v<=.04045,v/12.92,((v+.055)/1.055)**2.4)
print(json.dumps(dict(path=str(destination),sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),bytes=destination.stat().st_size,mean_rgb=v.mean(axis=(0,1)).tolist(),mean_roughness=(p[:,:,3]/255).mean(),mean_height_m=((p[:,:,4:].copy().view('<u2').reshape(512,512)/65535-.5)*.025).mean(),sources=checks),indent=2))
