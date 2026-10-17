"""Regression cases for the native/color/result-publication audit."""
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock
import numpy as np
from racelib.ackermann import CollisionWorld, hybrid_astar, planner_engine
from racelib.config import load_all, Config
from racelib.field import Field, Cone
from racelib.native_planner import c_hybrid_astar, is_native_available
from racelib.ai_vision import AIVisionEngine
from racelib.vision import Vision


class ColorContractTests(unittest.TestCase):
    def image(self,color):
        a=np.zeros((240,320,3),np.uint8)
        if color: a[50:190,80:240]=[26,84,214] if color=='blue' else [214,40,36]
        return a

    def test_consecutive_frames_and_reset(self):
        v=AIVisionEngine('fast_color')
        for c in ('blue',None,'blue','red',None,'red'):
            self.assertIsNone(v.detect_card(self.image(c)).route)
        self.assertEqual(v.detect_card(self.image('red')).route,'B')
        self.assertIsNone(v.detect_card(self.image(None)).route)

    def test_explicit_cpu_does_not_probe_hardware(self):
        with mock.patch('racelib.ai_vision.AscendNPUBackend',side_effect=AssertionError('must not probe')):
            self.assertEqual(AIVisionEngine('cpu','invalid.om').backend_name,'fast_vectorized')

    def test_resolution_scales_bearing(self):
        cfg=load_all()['vision']
        vision=Vision(cfg)
        vision.process(self.image(None),want_card=False)
        self.assertEqual((vision.cam.width,vision.cam.height),(320,240))
        self.assertAlmostEqual(vision.cam.bearing(160),0.)

    def test_npu_adapter_executes_and_rejects_wrong_shape(self):
        import sys
        from types import SimpleNamespace
        from racelib.ai_vision import AscendNPUBackend
        execute=mock.Mock(return_value=[np.array([[0.,1.,0.]],np.float32)])
        model=SimpleNamespace(_model_desc=1,execute=execute)
        mdl=SimpleNamespace(get_input_dims=lambda *_:({'dims':[1,3,224,224]},0),
            get_num_inputs=lambda *_:1,get_num_outputs=lambda *_:1,
            get_input_data_type=lambda *_:0,get_output_data_type=lambda *_:0,
            get_input_size_by_index=lambda *_:3*224*224*4,get_output_size_by_index=lambda *_:12)
        modules={'acl':SimpleNamespace(mdl=mdl),
            'acllite_resource':SimpleNamespace(AclLiteResource=lambda:SimpleNamespace(init=lambda:None)),
            'acllite_model':SimpleNamespace(AclLiteModel=lambda _:model)}
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(sys.modules,modules):
            path=Path(tmp)/'card.om';path.write_bytes(b'fake model for adapter test')
            backend=AscendNPUBackend(str(path))
            self.assertTrue(backend.is_available())
            self.assertEqual(backend.detect_card(self.image('blue')).route,'A')
            self.assertEqual(execute.call_count,2)
            self.assertEqual(execute.call_args.args[0][0].shape,(1,3,224,224))
            mdl.get_input_dims=lambda *_:({'dims':[1,3,640,640]},0)
            invalid=AscendNPUBackend(str(path))
            self.assertFalse(invalid.is_available())
            self.assertEqual(execute.call_count,2) # must reject before executing incompatible input

    def test_first_scan_uses_configured_start_before_init_handler(self):
        from racelib.ackermann import AckermannMission
        from racelib.lidarproc import ConeDetection,Scan
        from racelib.geometry import body_to_world
        from sim.io_sim import SimIO
        cfg=load_all();io=SimIO();mission=AckermannMission(io,cfg)
        start=(mission.field.start.cx,mission.field.start.cy,mission.field.start_heading)
        self.assertEqual(mission.pose(),start)
        io.put_scan(Scan(np.array([0.]),np.array([1.]),t=.01))
        detection=ConeDetection(1.,.5,.08,10,0.)
        with mock.patch('racelib.mission.extract_cones',return_value=[detection]):
            mission.step(.02)
        expected=body_to_world(start,(1.,.5))
        self.assertTrue(any(np.linalg.norm(t.xy-expected)<1e-8 for t in mission.cones.snapshot()))

    def test_no_card_timeout_stays_stopped(self):
        from racelib.ackermann import AckermannMission
        from racelib.mission import MissionConfig,State
        from sim.io_sim import SimIO
        cfg=load_all();m=MissionConfig.from_config(cfg['plan'],use_card=True)
        io=SimIO();mission=AckermannMission(io,cfg,m)
        mission.state=State.CARD;mission.state_t0=0
        io.advance(m.card_timeout_s+1)
        self.assertEqual(mission._st_card(.02),(0.,0.))
        self.assertEqual(mission.state,State.FAILED)

    def test_no_margin_relaxation_below_execution_guard(self):
        from racelib.ackermann import AckermannMission
        from sim.io_sim import SimIO
        mission=AckermannMission(SimIO(),load_all())
        mission.estimator.set_pose(1,2,0)
        mission.cones.snapshot=lambda:[Cone(1.365,2,'unknown',.115)]
        mission._rear_clearance=lambda:0.0   # no safe reverse available
        with mock.patch('racelib.ackermann.hybrid_astar',return_value=None) as search:
            self.assertEqual(mission._drive_to((4,2),.4),(0.,0.))
            # If the start cannot be unlocked, no search is made at all; if a
            # search is made, every attempted margin must stay above the guard.
            if search.called:
                self.assertGreaterEqual(search.call_args.args[2].margin,.06)

    def test_onnx_executes_exported_graph(self):
        try: import onnx; import onnxruntime
        except ImportError: self.skipTest('optional ONNX dependencies; run in vision image')
        from tools.export_model import export_model
        with tempfile.TemporaryDirectory() as tmp:
            path=export_model(Path(tmp)/'card.onnx')
            for color,route in [('blue','A'),('red','B')]:
                v=AIVisionEngine('onnx',str(path))
                self.assertEqual(v.backend_name,'onnxruntime')
                self.assertTrue(v.diagnostics['providers'])
                self.assertIsNone(v.detect_card(self.image(color)).route)
                self.assertEqual(v.detect_card(self.image(color)).route,route)
                self.assertIsNone(v.detect_card(self.image(None)).route)
            self.assertFalse(json.loads(path.with_suffix('.json').read_text())['trained'])


@unittest.skipUnless(is_native_available(),'build native planner first')
class NativeRegressionTests(unittest.TestCase):
    def field(self): return Field.from_config(load_all()['field'])
    def test_clockwise_boundary(self):
        f=self.field();f.boundary=[[0,0],[0,4],[7,4],[7,0]]
        w=CollisionWorld(f,[])
        path=c_hybrid_astar((1,2,0),(4,2),w)
        self.assertIsNotNone(path)
        self.assertTrue(all(w.clear(p) for p,_ in path))

    def test_large_measured_field_no_index_alias(self):
        f=self.field();f.length=30;f.width=10;f.boundary=[]
        w=CollisionWorld(f,[])
        path=c_hybrid_astar((14,7,0),(18,7),w)
        self.assertIsNotNone(path)
        self.assertLess(math.dist(path[-1][0][:2],(18,7)),.12)

    def test_clearance_matches_python_geometry(self):
        w=CollisionWorld(self.field(),[Cone(3,2,'unknown',.115)],margin=.06)
        rng=np.random.default_rng(123)
        for _ in range(150):
            p=(float(rng.uniform(.15,6.85)),float(rng.uniform(.15,3.85)),float(rng.uniform(-math.pi,math.pi)))
            path=c_hybrid_astar(p,p[:2],w,timeout=.01)
            self.assertEqual(path is not None,w.clear(p),msg=str(p))

    def test_public_timeout_and_actual_engine(self):
        w=CollisionWorld(self.field(),[Cone(5,1,'unknown',.3)])
        with mock.patch.dict(os.environ,{'CROSSFIRE_PLANNER_ENGINE':'native'}):
            self.assertEqual(planner_engine(),'cpp_native')
            start=time.monotonic()
            self.assertIsNone(hybrid_astar((1,1,0),(5,1),w,timeout=.02,max_nodes=100000))
            self.assertLess(time.monotonic()-start,.25)
        with mock.patch.dict(os.environ,{'CROSSFIRE_PLANNER_ENGINE':'python'}):
            self.assertEqual(planner_engine(),'python')

    def test_invalid_native_inputs_rejected(self):
        w=CollisionWorld(self.field(),[])
        for p in [(float('nan'),2,0),(2,2,float('inf'))]:
            self.assertIsNone(c_hybrid_astar(p,(4,2),w))


class ResultPublicationTests(unittest.TestCase):
    def test_done_summary_cannot_replace_final_truth_check(self):
        import shutil
        from tools.validate_simulations import check_run,ROOT
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp)
            shutil.copytree(ROOT/'config',run/'config')
            (run/'summary.json').write_text(json.dumps(dict(success=True,route='A',
                cone_contacts=0,wall_contacts=0,metrics=dict(route='A',planner_engine='cpp_native',hold_actual_s=3.))))
            (run/'events.json').write_text('[]');(run/'completed.json').write_text('{}')
            def trajectory(x,y):
                (run/'trajectory.csv').write_text('t,x,y,yaw,v,w,cmd_v,cmd_w\n1,'+str(x)+','+str(y)+',0,0,0,0,0\n')
            with mock.patch('tools.validate_simulations.subprocess.run',return_value=mock.Mock(returncode=0,stderr='')):
                trajectory(.45,2.) # DONE claim while still at start must fail
                self.assertFalse(check_run(run,0,'A',False,None)['passed'])
                trajectory(6.25,.75)
                self.assertTrue(check_run(run,0,'A',False,None)['passed'])

    def test_partial_video_does_not_replace_latest(self):
        from tools.sim_results import show_result
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'output';run=output/'new';run.mkdir(parents=True)
            (run/'summary.json').write_text(json.dumps({'route':'A','success':True}))
            (run/'video.partial.mp4').write_bytes(b'incomplete')
            self.assertFalse(show_result('/output/new',root=tmp))
            self.assertFalse((output/'latest').exists())
