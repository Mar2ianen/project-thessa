import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arena import png_size, sanitized_png
from cycles_b01 import compare_hdr, display_png, load_inputs, sha256


class CyclesB01Test(unittest.TestCase):
    def test_shared_display_convention_and_finite_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "display.png"
            display_png(path, [0, 1, 3, 1], 1, 1)
            self.assertEqual(png_size(path), (1, 1))
            sanitized_png(path, Path(directory) / "review.png")
            data = path.read_bytes()
            offset, compressed = 8, b""
            while offset < len(data):
                length, kind = struct.unpack_from(">I4s", data, offset)
                if kind == b"IDAT":
                    compressed += data[offset + 8:offset + 8 + length]
                offset += 12 + length
            self.assertEqual(zlib.decompress(compressed), bytes([0, 0, 188, 225, 255]))
            for rgba in [[0], [float("nan"), 0, 0, 1]]:
                with self.assertRaisesRegex(ValueError, "payload"):
                    display_png(path, rgba, 1, 1)

    def test_linear_metrics_do_not_hide_nonfinite_values_or_extent_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "native.rgba16f"
            path.write_bytes(struct.pack("<4e", 1, 2, 3, 1))
            self.assertEqual(compare_hdr(path, [1, 2, 3, 1])["relative_l2_against_thessa"], 0)
            metrics = compare_hdr(path, [2, 2, 3, 1])
            self.assertAlmostEqual(metrics["pre_exposed_linear_rgb_mae"], 1 / 3)
            self.assertAlmostEqual(metrics["relative_l2_against_thessa"], 1 / math.sqrt(14))
            with self.assertRaisesRegex(ValueError, "extent"):
                compare_hdr(path, [0] * 8)
            path.write_bytes(struct.pack("<4e", float("nan"), 2, 3, 1))
            with self.assertRaisesRegex(ValueError, "non-finite"):
                compare_hdr(path, [1, 2, 3, 1])

    def test_tampered_native_artifacts_fail_before_blender_import(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "scene.json").write_text("{}")
            (root / "scene.glb").write_bytes(b"original geometry")
            hashes = {name: sha256(root / name) for name in ("scene.json", "scene.glb")}
            (root / "capture.json").write_text(json.dumps({"artifacts_sha256": hashes}))
            (root / "scene.glb").write_bytes(b"substituted geometry")
            with self.assertRaisesRegex(ValueError, "artifact changed: scene.glb"):
                load_inputs(root)


if __name__ == "__main__":
    unittest.main()
