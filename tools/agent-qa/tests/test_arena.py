import contextlib
import io
import json
import struct
import zlib
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arena


def png(path, width=8, height=8):
    def chunk(name, data):
        return (struct.pack(">I", len(data)) + name + data +
                struct.pack(">I", zlib.crc32(name + data)))
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    scanlines = (bytes([0]) + bytes(width * 3)) * height
    path.write_bytes(arena.PNG_MAGIC + chunk(b"IHDR", header) +
                     chunk(b"tEXt", b"Software\x00Thessa") +
                     chunk(b"IDAT", zlib.compress(scanlines)) + chunk(b"IEND", b""))


class ArenaTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        png(self.root / "a.png")
        png(self.root / "b.png")
        self.manifest = self.root / "input.toml"
        self.manifest.write_text('''[[scene]]
id = "horizon"
first = "a.png"
first_label = "Thessa"
second = "b.png"
second_label = "Reference"
''')
        self.blind = self.root / "blind"
        self.private = self.root / "private"

    def test_blinding_and_votes(self):
        with contextlib.redirect_stdout(io.StringIO()):
            arena.prepare(self.manifest, self.blind, self.private, 42)
        public = (self.blind / "cases.json").read_text()
        self.assertNotIn("Thessa", public)
        self.assertNotIn("Reference", public)
        self.assertTrue((self.blind / "horizon" / "A.png").exists())
        self.assertNotIn(b"Thessa", (self.blind / "horizon" / "A.png").read_bytes())
        self.assertEqual(len(json.loads((self.private / "answer-key.json").read_text())), 1)
        arena.vote(self.blind, "horizon", "judge1", "tie", "same preference")
        with self.assertRaises(ValueError):
            arena.vote(self.blind, "horizon", "judge1", "A", "duplicate")
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            arena.score(self.blind, self.private / "answer-key.json")
        self.assertEqual(json.loads(buf.getvalue())["preference_points"],
                         {"Thessa": 0.5, "Reference": 0.5})

    def test_reject_leak_and_bad_image(self):
        with self.assertRaises(ValueError):
            arena.prepare(self.manifest, self.blind, self.blind / "key", 5)
        png(self.root / "b.png", width=7)
        with self.assertRaises(ValueError):
            arena.prepare(self.manifest, self.blind, self.private, 5)
        self.assertFalse(self.blind.exists())

    def test_unsafe_or_duplicate_case_id(self):
        self.manifest.write_text(self.manifest.read_text().replace("horizon", "../escape"))
        with self.assertRaises(ValueError):
            arena.prepare(self.manifest, self.blind, self.private, 5)


if __name__ == "__main__":
    unittest.main()
