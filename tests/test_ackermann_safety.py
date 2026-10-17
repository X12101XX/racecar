import math
import unittest
import numpy as np
from racelib.ackermann import integrate, CollisionWorld, hybrid_astar
from racelib.safety_policy import CommandPolicy
from racelib.config import load_all
from racelib.field import Field, Cone
from sim.world import SimWorld,SimRobotConfig,generate_cones

class TestAckermannSafety(unittest.TestCase):
    def test_yaw_rate_converts_to_steering(self):
        policy=CommandPolicy()
        v,delta=policy.convert(.4,.5,'legacy_steering')
        self.assertAlmostEqual(delta,math.atan(.3*.5/.4))
        self.assertEqual(policy.convert(0.,1.,'legacy_steering'),(0.,0.))
    def test_speed_limit_preserves_curvature(self):
        p=CommandPolicy(speed_limit=.3)
        v,w=p.convert(.6,.6,"yaw_rate")
        self.assertAlmostEqual(w/v,1.)

    def test_invalid_and_excessive_commands(self):
        p=CommandPolicy()
        self.assertEqual(p.convert(float('nan'),1.,'yaw_rate'),(0.,0.))
        self.assertEqual(p.convert(.2,float('inf'),'legacy_steering'),(0.,0.))
        v,s=p.convert(10.,100.,'legacy_steering')
        self.assertEqual(v,.5);self.assertEqual(s,.52)
        self.assertEqual(p.convert(-10.,0.,'legacy_steering')[0],-.2)
    def test_bicycle_cannot_spin(self):
        f=Field.from_config(load_all()['field']);w=SimWorld(f,[],np.random.default_rng(1),SimRobotConfig(drive_model='ackermann'))
        a=w.yaw;w.set_cmd(0.,1.)
        for _ in range(100):w.step(.01)
        self.assertAlmostEqual(w.yaw,a,places=12)
    def test_swept_rectangle_catches_between_endpoints(self):
        f=Field.from_config(load_all()['field']);w=CollisionWorld(f,[Cone(2.,2.,'unknown',.12)])
        self.assertTrue(w.clear((1.,2.,0.)))
        self.assertTrue(w.clear((3.,2.,0.)))
        self.assertFalse(w.edge((1.,2.,0.),2.,0.))
        self.assertFalse(w.clear((.1,2.,0.)))
    def test_hybrid_path_respects_car_and_obstacle(self):
        f=Field.from_config(load_all()['field']);w=CollisionWorld(f,[Cone(2.,2.,'unknown',.12)])
        path=hybrid_astar((1.,2.,0.),(3.,2.),w,timeout=8.)
        self.assertIsNotNone(path)
        self.assertTrue(all(w.clear(p) for p,_ in path))
        self.assertLess(math.hypot(path[-1][0][0]-3.,path[-1][0][1]-2.),.12)
    def test_wall_localization_after_turning(self):
        from racelib.localization import PoseEstimator
        f=Field.from_config(load_all()['field'])
        for yaw in (0.,math.pi/2,math.pi,-math.pi/2):
            w=SimWorld(f,[],np.random.default_rng(2));w.x,w.y,w.yaw=3.,2.,yaw
            est=PoseEstimator(f);est.set_pose(3.08,1.92,yaw)
            before=math.hypot(est.x-3.,est.y-2.)
            est.update_walls(w.lidar_scan(0.))
            self.assertLess(math.hypot(est.x-3.,est.y-2.),before)

    def test_official_counts_and_spacing(self):
        f=Field.from_config(load_all()['field'])
        for seed in range(30):
            cones=generate_cones(f,np.random.default_rng(seed));blue=[c for c in cones if c.color=='blue'];red=[c for c in cones if c.color=='red']
            self.assertTrue(12<=len(blue)<=16);self.assertTrue(5<=len(red)<=8)
            for i,x in enumerate(f.blockade_lines_x):
                line=sorted(c.y for c in blue if c.line==i)
                self.assertTrue(3<=len(line)<=6)
                self.assertTrue(all(b-a>=.4-1e-10 for a,b in zip(line,line[1:])))
                self.assertTrue(all(c.x==x for c in blue if c.line==i))

if __name__=='__main__':unittest.main()
