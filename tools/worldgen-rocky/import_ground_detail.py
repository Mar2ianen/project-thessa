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
    "diff": "93972c83249ae015111c8da714e6162c",
    "rough": "49bd9e53639081da4b8bda9163c469b5",
    "disp": "aee602747c99de3ecee2c4d969775aaf",
}
maps = {}
for name, digest in expected.items():
    path = source / f"forest_ground_04_{name}_2k.png"
    assert hashlib.md5(path.read_bytes()).hexdigest() == digest, path
    maps[name] = np.asarray(Image.open(path))


side = 2048
assert all(a.dtype == np.uint8 for k, a in maps.items() if k != "disp")
assert maps["disp"].dtype == np.uint16
assert maps["diff"].shape == (side, side, 3)
assert maps["rough"].shape == maps["disp"].shape == (side, side)
height = maps["disp"].astype("<u2")
packed = np.empty((side, side, 6), dtype=np.uint8)
packed[:, :, :3] = maps["diff"].astype(np.uint8)
packed[:, :, 3] = maps["rough"].astype(np.uint8)
packed[:, :, 4:] = height.view(np.uint8).reshape(side, side, 2)
raw = b"FG01" + side.to_bytes(4, "little") + packed.tobytes()
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_bytes(gzip.compress(raw, compresslevel=9, mtime=0))
metadata = {
    "path": str(destination),
    "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
    "bytes": destination.stat().st_size,
    "decoded_bytes": len(raw),
    "side": side,
    "source_url": "https://polyhaven.com/a/forest_ground_04",
    "license": "CC0-1.0",
    "width_m": 3.15,
    "height_range_m": 0.060,
    "height_is_authored_optical_prior": True,
    "sources": {
        name: {
            "url": f"https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/forest_ground_04/forest_ground_04_{name}_2k.png",
            "md5": digest,
            "sha256": hashlib.sha256((source / f"forest_ground_04_{name}_2k.png").read_bytes()).hexdigest(),
        } for name, digest in expected.items()
    },
}
destination.with_name(destination.name.removesuffix(".bin.gz") + ".provenance.json").write_text(
    json.dumps(metadata, indent=2) + "\n"
)
print(json.dumps(metadata))
