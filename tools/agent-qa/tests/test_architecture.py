import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_architecture import violations


class BoundaryTest(unittest.TestCase):
    def test_backend_and_copyleft_guard(self):
        packages = [
            {"id": "1", "name": "thessa-sim-core", "license": "MIT",
             "dependencies": [{"name": "wgpu"}, {"name": "thessa-game"}]},
            {"id": "2", "name": "thessa-game", "license": "GPL-3.0-or-later",
             "dependencies": []},
        ]
        self.assertEqual(violations({"packages": packages, "workspace_members": ["1", "2"]}), [
            "thessa-sim-core: MIT crate depends on GPL crate thessa-game",
            "thessa-sim-core: forbidden backend dependency wgpu",
        ])

    def test_clean_domain(self):
        packages = [{"id": "1", "name": "thessa-celestial", "license": "MIT",
                     "dependencies": [{"name": "glam"}]}]
        self.assertEqual(violations({"packages": packages, "workspace_members": ["1"]}), [])


if __name__ == "__main__":
    unittest.main()
