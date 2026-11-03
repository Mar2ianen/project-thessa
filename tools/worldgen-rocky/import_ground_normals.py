#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Pack pinned CC0 OpenGL normals. Usage: SCRIPT SOURCE_DIRECTORY OUTPUT_DIRECTORY.

The existing material swatches remain unchanged. Downsampling averages surface
slopes, then re-encodes RGB8. Runtime removes the matched height-map gradient.
Pillow's RGB conversion quantizes the original RGB16 maps to RGB8, as recorded.
"""
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

ASSETS = [
    ("forest_ground_04", "forest-ground-04", 1024, "77c2337c8c8c47357f69b598e73a72de"),
    ("brown_mud_leaves_01", "mud-litter-01", 512, "bfa4b76dc2218b8b953d3eb2055fafad"),
    ("forest_leaves_02", "forest-moss-litter-02", 1024, "bc9d39e13ec80f8db325f15e73372626"),
]

def encode_slopes(slopes):
    normal = np.concatenate((-slopes[..., :1], slopes[..., 1:2], np.ones_like(slopes[..., :1])), axis=-1)
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
    return np.rint((normal * .5 + .5) * 255).astype(np.uint8)

def pack(source, destination):
    destination.mkdir(parents=True, exist_ok=True)
    for asset, output, side, md5 in ASSETS:
        path = source / (asset + "-nor_gl-1k.png")
        raw = path.read_bytes()
        assert hashlib.md5(raw).hexdigest() == md5, path
        image = np.asarray(Image.open(path).convert("RGB"))
        assert image.shape == (1024, 1024, 3)
        folded = int(np.sum(image[..., 2] < 128))
        if side != 1024:
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
            "source_png_url": f"https://dl.polyhaven.org/file/ph-assets/Textures/png/1k/{asset}/{asset}_nor_gl_1k.png",
            "source_png_md5": md5, "source_png_sha256": hashlib.sha256(raw).hexdigest(),
            "path": str(target), "sha256": hashlib.sha256(packed).hexdigest(),
            "side": side, "bytes": len(packed), "format": "GN01 + u32 side + RGB8 OpenGL normal",
            "processing": "RGB16-to-RGB8 conversion; slope-area downsample only for the existing 512-square litter swatch",
            "nonpositive_z_source_texels": folded,
            "fallback": "A folded normal cannot be represented by a single-valued height graph; runtime retains its height-derived slope, not an infinite tangent gradient.",
        }
        target.with_name(output + ".normal.provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(output, len(packed))

if __name__ == "__main__":
    pack(Path(sys.argv[1]), Path(sys.argv[2]))
