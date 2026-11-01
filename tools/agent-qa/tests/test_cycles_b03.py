import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cycles_b03 import compare_hdr, display_png, load_inputs, sha256, invert_tonemap_pbr_neutral, tonemap_pbr_neutral


class CyclesB03Test(unittest.TestCase):
    def test_invert_tonemap_pbr_neutral_roundtrip(self):
        import numpy as np
        test_rgbs = [
            [0.1, 0.1, 0.1],
            [0.39, 0.31, 0.17],  # ground tone
            [0.63, 0.56, 0.51],  # sky tone
            [0.85, 0.80, 0.75],  # bright highlight
        ]
        for srgb_in in test_rgbs:
            arr = np.array([[srgb_in]], dtype=np.float32)
            v_pre = invert_tonemap_pbr_neutral(arr)
            out = tonemap_pbr_neutral(list(v_pre[0, 0]) + [1.0])
            expected = [int(round(c * 255.0)) for c in srgb_in]
            for o, e in zip(out[:3], expected):
                self.assertLessEqual(abs(o - e), 1)

    def test_shared_display_convention_and_finite_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "display.png"
            display_png(path, [0, 1, 3, 1], 1, 1)
            self.assertTrue(path.exists())
            self.assertGreater(path.stat().st_size, 0)
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

    def test_canonical_coast_scene_schema(self):
        scene_dir = Path(__file__).resolve().parents[3] / "data" / "reference-scenes" / "b03-canonical-coast"
        if (scene_dir / "scene.json").exists():
            data = json.loads((scene_dir / "scene.json").read_text())
            self.assertEqual(data["id"], "B03-canonical-coast-v1")
            self.assertIn("forward", data["camera"])
            self.assertIn("up", data["camera"])
            self.assertIn("right", data["camera"])
            self.assertGreater(data["stats"]["plant_instances"], 20000)
            self.assertGreater(data["stats"]["plant_triangles"], 100000000)


if __name__ == "__main__":
    unittest.main()
