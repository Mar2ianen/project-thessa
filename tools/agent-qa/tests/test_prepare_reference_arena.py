import contextlib
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arena
import prepare_reference_arena


def make_test_png(path: Path, width: int, height: int, color=(128, 128, 128)) -> None:
    img = Image.new("RGB", (width, height), color)
    img.save(path, format="PNG")


def make_test_jpg(path: Path, width: int, height: int, color=(200, 150, 100)) -> None:
    img = Image.new("RGB", (width, height), color)
    img.save(path, format="JPEG")


class PrepareReferenceArenaTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

        # Mock B01 dir
        self.b01_dir = self.root / "b01"
        self.b01_dir.mkdir()
        make_test_png(self.b01_dir / "thessa.png", 640, 480, (50, 50, 50))
        make_test_png(self.b01_dir / "display.png", 640, 480, (60, 60, 60))

        # Mock B02 dir
        self.b02_dir = self.root / "b02"
        self.b02_dir.mkdir()
        make_test_png(self.b02_dir / "thessa.png", 640, 480, (20, 100, 20))
        make_test_png(self.b02_dir / "display.png", 640, 480, (25, 105, 25))

        # Mock B03 Procedural dir
        self.b03_proc_dir = self.root / "b03_proc"
        self.b03_proc_dir.mkdir()
        make_test_png(self.b03_proc_dir / "thessa_display.png", 1280, 720, (100, 80, 40))
        make_test_png(self.b03_proc_dir / "cycles_display.png", 1280, 720, (102, 82, 44))

        # Mock B03 Golden dir
        self.b03_gold_dir = self.root / "b03_gold"
        self.b03_gold_dir.mkdir()
        make_test_jpg(self.b03_gold_dir / "golden_reference.jpg", 1920, 1080, (220, 140, 60))
        make_test_png(self.b03_gold_dir / "cycles_display.png", 1280, 720, (215, 138, 58))

        self.frames_dir = self.root / "frames"
        self.manifest_path = self.root / "arena-manifest.toml"
        self.blind_dir = self.root / "blind"
        self.private_dir = self.root / "private"

    def test_populate_and_manifest(self):
        prepare_reference_arena.populate_frames(
            self.frames_dir,
            self.b01_dir,
            self.b02_dir,
            self.b03_proc_dir,
            self.b03_gold_dir,
        )
        self.assertTrue((self.frames_dir / "b01_thessa.png").exists())
        self.assertTrue((self.frames_dir / "b01_cycles.png").exists())
        self.assertTrue((self.frames_dir / "b02_thessa.png").exists())
        self.assertTrue((self.frames_dir / "b02_cycles.png").exists())
        self.assertTrue((self.frames_dir / "b03_thessa.png").exists())
        self.assertTrue((self.frames_dir / "b03_cycles.png").exists())
        self.assertTrue((self.frames_dir / "b03_golden_ref.png").exists())
        self.assertTrue((self.frames_dir / "b03_golden_cycles.png").exists())

        self.assertEqual(arena.png_size(self.frames_dir / "b01_thessa.png"), (640, 480))
        self.assertEqual(arena.png_size(self.frames_dir / "b03_golden_ref.png"), (1280, 720))

        prepare_reference_arena.write_manifest(self.manifest_path, "frames")
        manifest_text = self.manifest_path.read_text(encoding="utf-8")
        self.assertIn("b01-material-lab", manifest_text)
        self.assertIn("b02-vegetation-cutout", manifest_text)
        self.assertIn("b03-canonical-coast", manifest_text)
        self.assertNotIn("b03-golden-sunset", manifest_text)

        # Ensure arena.prepare succeeds on this manifest
        with contextlib.redirect_stdout(io.StringIO()):
            arena.prepare(self.manifest_path, self.blind_dir, self.private_dir, 42)

        self.assertTrue((self.blind_dir / "cases.json").exists())
        self.assertTrue((self.private_dir / "answer-key.json").exists())

    def test_missing_input_raises(self):
        (self.b01_dir / "thessa.png").unlink()
        with self.assertRaises(FileNotFoundError):
            prepare_reference_arena.populate_frames(
                self.frames_dir,
                self.b01_dir,
                self.b02_dir,
                self.b03_proc_dir,
                self.b03_gold_dir,
            )


if __name__ == "__main__":
    unittest.main()
