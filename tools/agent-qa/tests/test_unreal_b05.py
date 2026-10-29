import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from unreal_b05 import camera_inputs, match_bounds, source_emission, to_unreal


class UnrealB05Test(unittest.TestCase):
    def test_capture_camera_converts_basis_metres_and_vertical_to_horizontal_fov(self):
        world = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,6,1]
        fy = 1/math.tan(math.radians(25))
        projection = [fy*3/4,0,0,0, 0,fy,0,0, 0,0,-1.001,-1, 0,0,-0.1001,0]
        values = {"capture_world_from_view":json.dumps(world), "capture_clip_from_view":json.dumps(projection)}
        result = camera_inputs(values)
        self.assertEqual(result["position_cm"],[0,600,0])
        self.assertEqual(result["forward"],[0,-1,0])
        self.assertEqual(result["right"],[1,0,0])
        self.assertEqual(result["up"],[0,0,1])
        self.assertAlmostEqual(math.tan(math.radians(result["horizontal_fov_degrees"])/2),1/projection[0])
        for changed in [projection[:3], [float("nan")]*16, [0]*16]:
            with self.assertRaisesRegex(ValueError,"perspective camera"):
                camera_inputs(values | {"capture_clip_from_view":json.dumps(changed)})
        world[0] = 2
        with self.assertRaisesRegex(ValueError,"orthonormal"):
            camera_inputs(values | {"capture_world_from_view":json.dumps(world)})

    def test_units_axes_and_finite_vectors_fail_explicitly(self):
        self.assertEqual(to_unreal([1,2,3],100),[100,300,200])
        for vector in [[0,0],[0,0,float("inf")]]:
            with self.assertRaisesRegex(ValueError,"three-component"):
                to_unreal(vector)

    def test_imported_panel_bounds_are_checked_independent_of_engine_naming(self):
        self.assertEqual(match_bounds([70,40,0],[130,0,130]),0)
        self.assertEqual(match_bounds([0,-60.0000038,0],[320,0,180]),1)
        self.assertEqual(match_bounds([-70,0,0],[130,0,130]),2)
        for center,extent in [([0,0,0],[130,0,130]), ([0,-60,0],[320,0,180.01])]:
            with self.assertRaisesRegex(ValueError,"bounds differ"):
                match_bounds(center,extent)

    def test_emission_mapping_is_literal_source_strength_not_unit_clamping(self):
        material = {"emissiveFactor":[0.1,0.5,0.9],
                    "extensions":{"KHR_materials_emissive_strength":{"emissiveStrength":16}}}
        self.assertEqual(source_emission(material)["radiance"],[1.6,8,14.4])
        self.assertEqual(source_emission({"emissiveFactor":[0.1,0.5,0.9]})["radiance"],[0.1,0.5,0.9])
        self.assertEqual(source_emission({})["radiance"],[0,0,0])
        for strength in [-1,float("inf"),float("nan")]:
            material["extensions"]["KHR_materials_emissive_strength"]["emissiveStrength"]=strength
            with self.assertRaisesRegex(ValueError,"invalid source"):
                source_emission(material)


if __name__ == "__main__":
    unittest.main()
