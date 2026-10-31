import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from plume_reference import CASES, ID, analytic_pixels, bounds, load_inputs, sha256, validate_scene


def fixture():
    return {"id":ID,"case":"axial-one","extent":[129,129],"units":"metres",
            "origin_len":[0,0,-5,4],"axis_radius":[0,0,-1,1.6],
            "profile":[[0,1,0.15,1,0.3,0.15,0.07,0],[4,1,0.15,1,0.3,0.15,0.07,0]],
            "steps":14,"exposure_gain":1,"erosion":0,"tail_dissolve":False,"diamonds":True,
            "world_from_view":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
            "clip_from_view":[2,0,0,0,0,2,0,0,0,0,-1.0000001,-1,0,0,-0.10000001,0]}


class PlumeReferenceTests(unittest.TestCase):
    def test_cylinder_carrier_bounds_follow_physical_axis(self):
        s=validate_scene(fixture())
        self.assertEqual(bounds(s),([0,0,-7],[1.6,1.6,2]))
        s["origin_len"]=[0,-2,-10,4];s["axis_radius"]=[0,1,0,1.6]
        self.assertEqual(bounds(s),([0,0,-10],[1.6,2,1.6]))

    def test_unsupported_fields_and_nonfinite_cameras_fail(self):
        for key,value in [("exposure_gain",2),("erosion",0.1),("tail_dissolve",True),("extent",[128,128])]:
            s=fixture();s[key]=value
            with self.assertRaises(ValueError):validate_scene(s)
        for key,index in [("axis_radius",0),("world_from_view",1),("clip_from_view",5)]:
            s=fixture();s[key][index]=math.nan
            with self.assertRaises(ValueError):validate_scene(s)

    def test_nonconstant_profile_and_nonunit_axis_are_rejected(self):
        s=fixture();s["profile"][1][4]=0.4
        with self.assertRaises(ValueError):validate_scene(s)
        s=fixture();s["axis_radius"][2]=-2
        with self.assertRaises(ValueError):validate_scene(s)

    def test_native_artifact_identity_is_verified_before_import(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);cases={}
            for case in CASES:
                p=root/"native"/case;p.mkdir(parents=True)
                scene=copy.deepcopy(fixture());scene["case"]=case
                (p/"scene.json").write_text(json.dumps(scene))
                (p/"linear.rgba16f").write_bytes(b"\x00\x3c"+bytes(129*129*8-2))
                (p/"display.png").write_bytes(b"fixture")
                (p/"analytic.png").write_bytes(b"fixture")
                (p/"analytic.json").write_bytes(b"fixture")
                cases[case]={f.name:sha256(f) for f in p.iterdir()}
            (root/"capture.json").write_text(json.dumps({"id":ID,"cases":cases}))
            load_inputs(root)
            (root/"native"/CASES[0]/"scene.json").write_text("{}")
            with self.assertRaises(ValueError):load_inputs(root)

    def test_gaussian_line_integral_matches_known_axial_and_side_solutions(self):
        scene=fixture(); image=analytic_pixels(scene);center=(64*129+64)*4
        alpha=-math.expm1(-0.15*4*1.18)
        self.assertAlmostEqual(image[center],0.3/0.15*alpha,places=12)
        self.assertAlmostEqual(image[center+3],alpha,places=12)
        self.assertEqual(image[:4],[0]*4)
        scene["profile"][0][2]=scene["profile"][1][2]=0
        image=analytic_pixels(scene)
        self.assertAlmostEqual(image[center],0.3*1.18*4,places=12)
        self.assertEqual(image[center+3],0)
        scene=fixture();scene["origin_len"]=[0,-2,-10,4];scene["axis_radius"]=[0,1,0,1.6]
        image=analytic_pixels(scene)
        chord=math.sqrt(math.pi/2.2)*math.erf(1.6*math.sqrt(2.2))+0.18*math.sqrt(math.pi/0.7)*math.erf(1.6*math.sqrt(0.7))
        self.assertAlmostEqual(image[center],2*(-math.expm1(-0.15*chord)),places=12)


if __name__=="__main__":unittest.main()
