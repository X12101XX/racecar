"""Independent camera geometry and front-camera-compatible route checks."""
import math
from types import SimpleNamespace
import unittest
import numpy as np
from racelib.config import load_all
from racelib.field import Field, Cone
from racelib.ackermann import CollisionWorld, hybrid_astar
from racelib.vision import CameraModel
from sim.camera import CameraRenderer


class TestCameraHeading(unittest.TestCase):
    def setUp(self):
        self.field=Field.from_config(load_all()['field'])
        self.cam=CameraModel(width=320,height=240)
        self.renderer=CameraRenderer(self.field,self.cam,noise_sigma=0,blur=False,monochrome=True)

    def test_left_and_right_pixels_at_four_body_headings(self):
        for yaw in (0.,math.pi/2,math.pi,-math.pi/2):
            pose=(3.,2.,yaw);base=self.renderer.render([],pose)
            for side in (-1,1):
                # Place a cone 1.66 m forward and 0.45 m to the body's left/right.
                cone=SimpleNamespace(x=3+1.66*math.cos(yaw)-side*.45*math.sin(yaw),
                                     y=2+1.66*math.sin(yaw)+side*.45*math.cos(yaw),radius_base=.115)
                image=self.renderer.render([cone],pose)
                ys,xs=np.where(np.any(image!=base,axis=2))
                self.assertGreater(len(xs),50)
                self.assertLess(side*(float(xs.mean())-self.cam.cx),0)

    def test_behind_camera_is_invisible(self):
        pose=(3.,2.,0.)
        behind=SimpleNamespace(x=1.5,y=2.,radius_base=.115)
        np.testing.assert_array_equal(self.renderer.render([behind],pose),self.renderer.render([],pose))

    def test_projected_ground_matches_distance_model(self):
        mount=np.array([3.16,2.,.18]);pitch=math.radians(self.cam.pitch_deg)
        points=np.array([[3.16+distance,2.,0.] for distance in (.5,1.,2.,4.)])
        pixels=self.renderer._project(self.renderer._to_camera(points,mount,0.,pitch))
        for distance,(_,row) in zip((.5,1.,2.,4.),pixels):
            self.assertAlmostEqual(self.cam.ground_distance(row),distance,places=8)
        far=np.array([[1000003.16,2.,0.]])
        row=self.renderer._project(self.renderer._to_camera(far,mount,0.,pitch))[0,1]
        self.assertAlmostEqual(row,self.cam.horizon_row(),places=3)

    def test_crossing_prefers_forward_when_feasible(self):
        self.field.length,self.field.width=7.4,4.2
        points=[(2,.45),(2,1.25),(2.54,1.62),(2.17,3.7),(3.65,.45),
                (3.46,3.66),(1.94,2.08),(2.71,3.58),(5.15,.45),(5.08,2.44),
                (4.96,2.96),(4.39,3.46),(2.7,.8),(3.45,1.59),(4.2,1.1),(4.12,2.96),(4.65,.5)]
        world=CollisionWorld(self.field,[Cone(x,y,'unknown',.115) for x,y in points],margin=.1)
        path=hybrid_astar((.79,3.1,1.9),(6.25,.75),world)
        self.assertIsNotNone(path)
        self.assertTrue(all(gear==1 for _,gear in path))
        self.assertTrue(all(world.clear(p) for p,_ in path))

    def test_reverse_fallback_has_a_cumulative_distance_budget(self):
        self.field.width=.8
        world=CollisionWorld(self.field,[],margin=.06)
        path=hybrid_astar((3.,.4,0.),(2.56,.4),world,max_reverse_m=.44,prefer_forward=False)
        self.assertIsNotNone(path)
        reverse=sum(math.dist(a[0][:2],b[0][:2]) for a,b in zip(path,path[1:]) if b[1]<0)
        self.assertGreater(reverse,.3)
        self.assertLessEqual(reverse,.440001)
        self.assertIsNone(hybrid_astar((3.,.4,0.),(2.,.4),world,timeout=.5,
                                      max_reverse_m=.44,prefer_forward=False))


if __name__=='__main__':unittest.main()
