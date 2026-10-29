import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture import load_manifest, validate_capture, reference_identity


class CaptureTest(unittest.TestCase):
    def test_input_checksum_is_enforced_without_fetching(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "input").write_bytes(b"fixed data")
            manifest = root / "scene.toml"
            manifest.write_text('schema_version=1\nid="probe"\nwidth=7\nheight=3\nframes=1\n'
                                '[[inputs]]\npath="input"\nsha256="' + hashlib.sha256(b"fixed data").hexdigest() + '"\n')
            self.assertEqual(load_manifest(manifest, root)[0]["id"], "probe")
            (root / "input").write_bytes(b"different data")
            with self.assertRaisesRegex(ValueError, "checksum mismatch"):
                load_manifest(manifest, root)

    def test_output_and_workload_flags_cannot_override_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.toml"
            for tail in ['native_args=["--frames", "999"]', 'native_args=["--headless=elsewhere.png"]', 'native_args=[1]']:
                path.write_text('schema_version=1\nid="probe"\nwidth=7\nheight=3\nframes=1\n' + tail)
                with self.assertRaises(ValueError):
                    load_manifest(path)
            path.write_text('schema_version=1\nid="probe"\nwidth=0\nheight=3\nframes=1\n')
            with self.assertRaisesRegex(ValueError, "width"):
                load_manifest(path)

    def test_reference_definition_and_geometry_are_bound_to_actual_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "scene.json").write_text('{"id":"B01"}')
            (root / "scene.glb").write_bytes(b"geometry")
            capture = {"metadata": {"scenario": "B01", "graphics": {"resolved": {
                "reference_scene_file": "scene.json", "reference_geometry_file": "scene.glb"}}}}
            result = reference_identity(root, {"id": "B01"}, capture)
            self.assertEqual(result["reference_geometry_sha256"], hashlib.sha256(b"geometry").hexdigest())
            with self.assertRaisesRegex(ValueError, "identity"):
                reference_identity(root, {"id": "different"}, capture)
            capture["metadata"]["graphics"]["resolved"]["reference_geometry_file"] = "../outside"
            with self.assertRaisesRegex(ValueError, "filename"):
                reference_identity(root, {"id": "B01"}, capture)

    def test_assets_cannot_enter_without_explicit_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "asset").write_bytes(b"asset")
            digest = hashlib.sha256(b"asset").hexdigest()
            manifest = root / "scene.toml"
            manifest.write_text('schema_version=1\nid="probe"\nwidth=7\nheight=3\nframes=1\n'
                                '[[assets]]\npath="asset"\nsha256="' + digest + '"\n')
            with self.assertRaisesRegex(ValueError, "incomplete asset provenance"):
                load_manifest(manifest, root)

    def test_capture_requires_actual_identity_camera_light_and_warmup(self):
        values = {name: "explicit" for name in (
            "capture_exposure_gain", "capture_simulation_time_s", "capture_ambient_lux",
            "capture_light_directions", "capture_light_colors_lux", "capture_projection_convention",
            "capture_linear_hdr", "requested_toml", "measurement_boundary")}
        values.update(capture_world_from_view=json.dumps([0] * 16),
                      capture_clip_from_view=json.dumps([0] * 16),
                      capture_camera_origin_m="[0,0,0]", capture_warmup_frames="30")
        manifest = {"width": 7, "height": 3, "frames": 1}
        data = {"metadata": {"build": {"commit": "source"}, "display": {"width": 7, "height": 3},
                             "graphics": {"resolved": values}}, "frames": [{}]}
        validate_capture(data, manifest, "source")
        for key, value in [("capture_world_from_view", "[0]"), ("capture_camera_origin_m", "[NaN,0,0]"),
                           ("capture_warmup_frames", "0")]:
            broken = copy.deepcopy(data)
            broken["metadata"]["graphics"]["resolved"][key] = value
            with self.assertRaises(ValueError):
                validate_capture(broken, manifest, "source")
        with self.assertRaisesRegex(ValueError, "source label"):
            validate_capture(data, manifest, "different")


if __name__ == "__main__":
    unittest.main()
