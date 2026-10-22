#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Pack verified CC0 source maps; Pillow/NumPy are offline tooling only."""
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
    "diff": "8f4318ba5044aa147999b08a10302fb4",
    "rough": "d9a63bc30621bf8edd1a3a44d2245a16",
    "disp": "fb41f8f219e76d6a52095f12bb3c20b2",
}
maps = {}
for name, digest in expected.items():
    path = source / f"forest_ground_04_{name}_1k.png"
    assert hashlib.md5(path.read_bytes()).hexdigest() == digest, path
    maps[name] = np.asarray(Image.open(path)).astype(np.float64)


def average(a):
    return (a[0::2, 0::2] + a[1::2, 0::2] + a[0::2, 1::2] + a[1::2, 1::2]) * 0.25


rgb = maps["diff"] / 255.0
linear = np.where(rgb <= 0.04045, rgb / 12.92, ((rgb + 0.055) / 1.055) ** 2.4)
linear = average(linear)
rgb = np.where(linear <= 0.0031308, linear * 12.92, 1.055 * linear ** (1 / 2.4) - 0.055)
rough = np.rint(average(maps["rough"])).astype(np.uint8)
height = np.rint(average(maps["disp"])).astype("<u2")
packed = np.empty((512, 512, 6), dtype=np.uint8)
packed[:, :, :3] = np.rint(rgb * 255).astype(np.uint8)
packed[:, :, 3] = rough
packed[:, :, 4:] = height.view(np.uint8).reshape(512, 512, 2)
raw = b"FG01" + (512).to_bytes(4, "little") + packed.tobytes()
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_bytes(gzip.compress(raw, compresslevel=9, mtime=0))
print(json.dumps({"path": str(destination), "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
                  "bytes": destination.stat().st_size, "decoded_bytes": len(raw)}))
