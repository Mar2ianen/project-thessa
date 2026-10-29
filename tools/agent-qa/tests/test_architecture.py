import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_architecture import inspect, violations


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


def graph(edges, *, licenses=None, external=()):
    """Cargo-shaped resolved graph; dependency aliases need not match package names."""
    licenses = licenses or {}
    names = set(edges) | {target for deps in edges.values() for target, _ in deps}
    packages = [{"id": name, "name": name, "license": licenses.get(name, "MIT"),
                 "dependencies": [{"name": target, "kind": kind}
                                  for target, kind in edges.get(name, [])]}
                for name in sorted(names)]
    nodes = [{"id": name, "deps": [
        {"name": "renamed_dependency", "pkg": target,
         "dep_kinds": [{"kind": kind, "target": "cfg(unix)"}]}
        for target, kind in edges.get(name, [])]} for name in sorted(names)]
    return {"packages": packages, "workspace_members": sorted(names - set(external)),
            "resolve": {"nodes": nodes}}


class ResolvedBoundaryTest(unittest.TestCase):
    def test_indirect_renamed_target_build_backend(self):
        data = graph({"thessa-render-core": [("bridge", None)],
                      "bridge": [("wgpu-core", "build")]}, external={"bridge", "wgpu-core"})
        self.assertEqual(violations(data), [
            "thessa-render-core: backend dependency path: thessa-render-core -> bridge -> wgpu-core"])

    def test_indirect_copyleft(self):
        data = graph({"engine": [("bridge", None)], "bridge": [("game", None)]},
                     licenses={"game": "GPL-3.0-or-later"})
        self.assertIn("engine: MIT-to-GPL dependency path: engine -> bridge -> game", violations(data))

    def test_own_dev_dependency_is_checked(self):
        data = graph({"thessa-render-core": [("test-bridge", "dev")],
                      "test-bridge": [("wgpu", None)]}, external={"test-bridge", "wgpu"})
        self.assertTrue(any("backend dependency path" in e for e in violations(data)))

    def test_dependency_dev_edges_are_not_inherited(self):
        data = graph({"thessa-render-core": [("library", None)],
                      "library": [("wgpu", "dev")]}, external={"library", "wgpu"})
        self.assertEqual(violations(data), [])

    def test_renderer_must_not_require_game_worldgen(self):
        data = graph({"thessa-render-wgpu": [("bridge", None)],
                      "bridge": [("thessa-worldgen-rocky", None)]})
        self.assertIn("thessa-render-wgpu: game terrain is mandatory for rendering: "
                      "thessa-render-wgpu -> bridge -> thessa-worldgen-rocky", violations(data))

    def test_renderer_test_scene_is_not_a_runtime_requirement(self):
        data = graph({"thessa-render-wgpu": [("thessa-worldgen-rocky", "dev")]})
        self.assertEqual(violations(data), [])

    def test_library_cannot_depend_on_an_application(self):
        data = graph({"engine": [("bridge", None)], "bridge": [("thessa-client", None)]})
        self.assertTrue(any("engine: reverse application dependency" in e for e in violations(data)))

    def test_staging_is_an_exact_edge_not_a_root_exemption(self):
        data = graph({"thessa-fuselage": [("thessa-sim-core", None)]})
        result = inspect(data)
        self.assertEqual(result["errors"], [])
        self.assertEqual(len(result["staged"]), 1)
        data = graph({"thessa-fuselage": [("thessa-sim-core", None), ("new-bridge", None)],
                      "new-bridge": [("thessa-celestial", None)]})
        self.assertTrue(any("generic physics requires" in e for e in violations(data)))

    def test_new_generic_physics_celestial_path_is_rejected(self):
        data = graph({"thessa-collision": [("new-bridge", None)],
                      "new-bridge": [("thessa-celestial", None)]})
        self.assertTrue(any("generic physics requires" in e for e in violations(data)))

    def test_collision_cannot_restore_the_removed_facade_edge(self):
        data = graph({"thessa-collision": [("thessa-sim-core", None)]})
        self.assertTrue(any("generic physics requires" in e for e in violations(data)))

    def test_cycle_terminates_and_keeps_shortest_diagnostic(self):
        data = graph({"thessa-render-core": [("bridge", None)],
                      "bridge": [("thessa-render-core", None), ("wgpu", None)]},
                     external={"bridge", "wgpu"})
        self.assertEqual(len(violations(data)), 1)


if __name__ == "__main__":
    unittest.main()
