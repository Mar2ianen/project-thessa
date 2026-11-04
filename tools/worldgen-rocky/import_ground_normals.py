#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Pack pinned CC0 OpenGL normals. Usage: SCRIPT SOURCE_DIRECTORY OUTPUT_DIRECTORY.

Matched material swatches own the source dimensions. Downsampling averages surface
slopes, then re-encodes RGB8. Runtime removes the matched height-map gradient.
Native RGB8 is preserved; RGB16 sources are converted to RGB8, as recorded.
"""
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

ASSETS = [
    ("forest_ground_04", "forest-ground-04", 2048, "2k", "c7bf8a98de1dee39cb6028e06b8bdd37"),
    ("brown_mud_leaves_01", "mud-litter-01", 1024, "1k", "bfa4b76dc2218b8b953d3eb2055fafad"),
    ("forest_leaves_02", "forest-moss-litter-02", 2048, "2k", "cce2a8fd9454483a17f65b3cfc832725"),
]

def encode_slopes(slopes):
    normal = np.concatenate((-slopes[..., :1], slopes[..., 1:2], np.ones_like(slopes[..., :1])), axis=-1)
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
    return np.rint((normal * .5 + .5) * 255).astype(np.uint8)

def pack(source, destination):
    destination.mkdir(parents=True, exist_ok=True)
    for asset, output, side, resolution, md5 in ASSETS:
        path = source / (asset + "-nor_gl-" + resolution + ".png")
        raw = path.read_bytes()
        assert hashlib.md5(raw).hexdigest() == md5, path
        image = np.asarray(Image.open(path).convert("RGB"))
        source_side = 2048 if resolution == "2k" else 1024
        assert image.shape == (source_side, source_side, 3)
        folded = int(np.sum(image[..., 2] < 128))
        if side != source_side:
            n = image.astype(float) / 255 * 2 - 1
            assert np.all(n[..., 2] > 0), "downsample requires representable source normals"
            slope = np.stack((-n[..., 0]/n[..., 2], n[..., 1]/n[..., 2]), axis=-1)
            slope = slope.reshape(side, 2, side, 2, 2).mean(axis=(1, 3))
            image = encode_slopes(slope)
        payload = b"GN01" + side.to_bytes(4, "little") + image.tobytes()
        packed = gzip.compress(payload, compresslevel=9, mtime=0)
        target = destination / (output + ".normal.bin.gz")
        target.write_bytes(packed)
        metadata = {
            "license": "CC0-1.0", "source_url": "https://polyhaven.com/a/" + asset,
            "source_png_url": f"https://dl.polyhaven.org/file/ph-assets/Textures/png/{resolution}/{asset}/{asset}_nor_gl_{resolution}.png",
            "source_png_md5": md5, "source_png_sha256": hashlib.sha256(raw).hexdigest(),
            "path": str(target), "sha256": hashlib.sha256(packed).hexdigest(),
            "side": side, "bytes": len(packed), "format": "GN01 + u32 side + RGB8 OpenGL normal",
            "source_png_bit_depth": raw[24],
            "processing": ("native RGB8" if raw[24] == 8 else "RGB16-to-RGB8 conversion") + ("; slope-area downsample" if side != source_side else "; native lattice"),
            "nonpositive_z_source_texels": folded,
            "fallback": "A folded normal cannot be represented by a single-valued height graph; runtime retains its height-derived slope, not an infinite tangent gradient.",
        }
        target.with_name(output + ".normal.provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(output, len(packed))

if __name__ == "__main__":
    pack(Path(sys.argv[1]), Path(sys.argv[2]))
