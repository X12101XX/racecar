"""Meaningful geometry and profile propagation checks; no car motion."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from profile_config import default_profile,validate,compile_profile,save_profile
from racelib.settings import MOTION_PRESETS,motion_settings
from racelib.config import load_all,Config
from racelib.field import Field
from racelib.ackermann import AckermannMission,CollisionWorld
from racelib.localization import PoseEstimator
from sim.io_sim import SimIO
from sim.world import SimWorld,SimRobotConfig

class TestSceneConfig(unittest.TestCase):
    def test_nonstandard_dimensions_are_warnings(self):
        p=default_profile();p['field']['arena']['length_m']=9.2;p['field']['minefield']['blockade_lines_x']=[2.,3.7,5.1]
        r=validate(p);self.assertTrue(r['valid'],r);self.assertTrue(r['warnings'])
    def test_crossed_fence_and_outside_zone_rejected(self):
        p=default_profile();p['field']['arena']['boundary']=[[0,0],[7,4],[7,0],[0,4]]
        self.assertFalse(validate(p)['valid'])
        p=default_profile();p['field']['zones']['camp_a']['center']=[7.,.75]
        self.assertFalse(validate(p)['valid'])
    def test_invalid_motion_and_tiny_zone_rejected(self):
        for key,value in [('forward_mps',float('nan')),('brake_mps2',1.5),('planning_margin_m',0.)]:
            p=default_profile();p['motion'][key]=value;self.assertFalse(validate(p)['valid'])
        p=default_profile();p['field']['zones']['camp_a']['size']=[.3,.3];self.assertFalse(validate(p)['valid'])
    def test_export_separates_truth_and_known_obstacles(self):
        p=default_profile();p['cones'][0]['known']=True;p['motion']=MOTION_PRESETS['aggressive'].copy()
        with tempfile.TemporaryDirectory() as d:
            compile_profile(p,d);cfg=load_all(d,'A');field=Field.from_config(cfg['field'])
            self.assertEqual(len(field.prior_cones()),1)
            self.assertEqual(len(json.loads((Path(d)/'sim_cones.json').read_text())),17)
            self.assertEqual(motion_settings(cfg['plan'])['forward_mps'],.65)
            self.assertFalse(json.loads((Path(d)/'vehicle_profile.json').read_text())['hardware_watchdog_confirmed'])
    def test_save_is_versioned_and_cannot_escape_folder(self):
        with tempfile.TemporaryDirectory() as d:
            p=default_profile();save_profile('site',p,d);p['motion']['forward_mps']=.45;save_profile('site',p,d)
            self.assertEqual(len(list((Path(d)/'history').glob('*.json'))),1)
            with self.assertRaises(ValueError):save_profile('../outside',p,d)
    def test_requested_speed_reaches_real_controller(self):
        outputs=[]
        for requested in (.2,.65):
            cfg=load_all(route='A');m=MOTION_PRESETS['standard'].copy();m.update(forward_mps=requested,corner_mps=min(.25,requested),reverse_mps=min(.2,requested))
            cfg['plan'].data['ackermann']=m
            node=AckermannMission(SimIO(),cfg);node.estimator.set_pose(1.,2.,0.)
            node._segment=[(x,2.,0.) for x in np.linspace(1.,4.,100)];node._last_goal=np.array([4.,2.]);node._next_index=0;node._direction=1;node._speed=1.;node._last_dt=.02
            v,w=node._drive_to([4.,2.],1.)
            outputs.append(v);self.assertAlmostEqual(v,requested);self.assertAlmostEqual(w,0.)
        self.assertGreater(outputs[1],outputs[0])
    def test_slanted_wall_changes_rays_and_swept_footprint(self):
        p=default_profile();p['field']['arena']['boundary']=[[.1,.1],[6.9,.2],[6.7,3.9],[.2,3.8]]
        field=Field.from_config(Config(p['field']));world=SimWorld(field,[],np.random.default_rng(1))
        world.x,world.y,world.yaw=3.,2.,0.;scan=world.lidar_scan(0.)
        front=scan.ranges[np.argmin(abs(scan.angles))];self.assertAlmostEqual(front,3.8,delta=.08)
        self.assertTrue(CollisionWorld(field,[]).clear((3.,2.,0.)))
        self.assertFalse(CollisionWorld(field,[]).clear((6.8,3.,0.)))
        est=PoseEstimator(field);est.set_pose(3.08,1.92,0.)
        before=np.linalg.norm(np.array(est.pose[:2])-[3.,2.]);est.update_walls(scan)
        self.assertLess(np.linalg.norm(np.array(est.pose[:2])-[3.,2.]),before)

if __name__=='__main__':unittest.main()
