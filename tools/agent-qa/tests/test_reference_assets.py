import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import reference_assets as assets


class ReferenceAssetsTest(unittest.TestCase):
    def test_cache_paths_cannot_escape_through_paths_or_symlinks(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cache = root / "cache"
            cache.mkdir()
            (cache / "link").symlink_to(root, target_is_directory=True)
            for path in ["../outside", str(root / "absolute"), "link/outside", "."]:
                with self.subTest(path=path), self.assertRaisesRegex(ValueError, "escapes"):
                    assets.cache_path(cache, path)

    def test_fetch_is_bounded_verified_and_reuses_only_verified_data(self):
        data = b"pinned bytes"
        item = {"path": "raw/asset", "url": "https://dl.polyhaven.org/file/test",
                "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory)
            with patch.object(assets.urllib.request, "urlopen", return_value=io.BytesIO(data)) as fetch:
                path = assets.fetch_source(cache, item)
                self.assertEqual(path.read_bytes(), data)
                self.assertIn("ProjectThessa", fetch.call_args.args[0].get_header("User-agent"))
            with patch.object(assets.urllib.request, "urlopen") as fetch:
                self.assertEqual(assets.fetch_source(cache, item), path)
                path.write_bytes(b"corrupt")
                with self.assertRaisesRegex(ValueError, "existing source checksum"):
                    assets.fetch_source(cache, item)
                fetch.assert_not_called()

    def test_failed_download_never_installs_an_artifact(self):
        data = b"pinned bytes"
        item = {"path": "raw/asset", "url": "https://dl.polyhaven.org/file/test",
                "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory)
            for response in [b"short", b"wrong bytes!", data + b"extra"]:
                with self.subTest(response=response):
                    with patch.object(assets.urllib.request, "urlopen", return_value=io.BytesIO(response)):
                        with self.assertRaisesRegex(ValueError, "download checksum"):
                            assets.fetch_source(cache, item)
                    self.assertFalse((cache / item["path"]).exists())
            for change in [{"url": "https://other.invalid/file"}, {"bytes": 0}, {"bytes": (16 << 20) + 1}]:
                with patch.object(assets.urllib.request, "urlopen") as fetch:
                    with self.assertRaisesRegex(ValueError, "host or file budget"):
                        assets.fetch_source(cache, item | change)
                    fetch.assert_not_called()

    def test_manifest_pins_and_cached_artifacts_are_enforced_before_conversion(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "source"
            cache = Path(directory) / "cache"
            root.mkdir()
            cache.mkdir()
            (root / "converter.py").write_bytes(b"trusted converter")
            (root / "license.txt").write_bytes(b"license notice")
            (cache / "model.glb").write_bytes(b"model")
            manifest = {"schema_version": 1,
                        "converter": {"path": "converter.py", "sha256": assets.sha256(root / "converter.py")},
                        "license_text": {"sha256": assets.sha256(root / "license.txt")},
                        "assets": [{"id": "probe", "path": "model.glb", "license_notice": "license.txt",
                                    "converted_sha256": assets.sha256(cache / "model.glb")}]}
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest))
            with patch.object(assets, "ROOT", root), patch.object(assets.subprocess, "run") as blender:
                assets.prepare(manifest_path, cache)
                with self.assertRaisesRegex(ValueError, "outside"):
                    assets.prepare(manifest_path, root / "cache")
                (cache / "model.glb").write_bytes(b"corrupt")
                with self.assertRaisesRegex(ValueError, "converted asset checksum"):
                    assets.prepare(manifest_path, cache)
                (root / "license.txt").write_bytes(b"changed license")
                with self.assertRaisesRegex(ValueError, "license notice"):
                    assets.prepare(manifest_path, cache)
                (root / "converter.py").write_bytes(b"changed converter")
                with self.assertRaisesRegex(ValueError, "converter changed"):
                    assets.prepare(manifest_path, cache)
                blender.assert_not_called()


if __name__ == "__main__":
    unittest.main()
